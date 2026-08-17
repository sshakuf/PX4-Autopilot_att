#!/usr/bin/env python3
"""
Position-Mode PID Tuner
=======================

Interactive MAVLink console to tune the SpinAir horizontal drone for SMOOTH
hover / translation while running in Position mode with Direct Flight OFF and
Yaw hold ON.

Workflow:
  1. On startup it establishes the baseline config (unless --no-setup):
        DF_MC_DIR_EN   = 0   (Direct Flight off)
        DF_YAW_HOLD_EN = 1   (yaw hold on)
        DF_YAWSPD_PID_EN = 1 (yaw-speed PID on)
     and switches the vehicle to POSCTL.
  2. You edit 4 param groups live (PARAM_SET + read-back verify):
        - Accel->thrust        : DF_ACC_PER_THR
        - Horizontal vel PID   : MPC_XY_P, MPC_XY_VEL_P_ACC/_I_ACC/_D_ACC
        - Yaw hold             : DF_YAWSPEED_P/I/D, DF_YAW_FINE_P/I/D, +limits
        - Payload scaling      : DF_PAYLOAD_KG, DF_PAYLOAD_MIN
  3. You run small, bounded stick maneuvers and read a smoothness scorecard:
        - hold/observe : steady-state jitter (the "camera jumping" measure)
        - nudge&release: overshoot / oscillation / settling after a small step
        - yaw step     : yaw settling after a small heading change
  4. Export a working set to logs/run_<ts>/tuned.params.

!!! SAFETY: this arms a real drone and spins the props. Secure the drone on its
    wire. Any key during a trial = immediate force-disarm. Ctrl-C / crash / exit
    all force-disarm and restore the terminal.

The prime suspect for "jumping" on the bigger drone is DF_ACC_PER_THR: its own
docs say bigger fans want ~3.0 (default 0.5). Too low => ~6x too much thrust per
commanded acceleration => saturation / oscillation. Tune it first.

See docs/superpowers/specs/2026-07-04-position-mode-pid-tuner-design.md
"""

import argparse
import csv
import json
import math
import os
import select
import signal
import struct
import sys
import termios
import threading
import time
import tty
from datetime import datetime

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink not installed.  pip install -r requirements.txt")


# --------------------------------------------------------------------------- #
# Tunable parameters: groups, ordering, and integer-vs-float typing
# --------------------------------------------------------------------------- #
# Groups edited via the 'p' menu. Order here is the display + trials.csv order.
PARAM_GROUPS = {
    "acc": ["DF_ACC_PER_THR"],
    "xyvel": ["MPC_XY_P", "MPC_XY_VEL_P_ACC", "MPC_XY_VEL_I_ACC", "MPC_XY_VEL_D_ACC"],
    "yaw": ["DF_YAWSPEED_P", "DF_YAWSPEED_I", "DF_YAWSPEED_D",
            "DF_YAW_FINE_P", "DF_YAW_FINE_I", "DF_YAW_FINE_D",
            "DF_YAWSPEED_MAXR", "DF_YAW_ACC_MAX"],
    "payload": ["DF_PAYLOAD_KG", "DF_PAYLOAD_MIN"],
}
GROUP_TITLES = {
    "acc": "Accel->thrust",
    "xyvel": "Horizontal velocity PID",
    "yaw": "Yaw hold PIDs",
    "payload": "Payload scaling",
}
# Flat, ordered list of every tunable param (trials.csv columns + export order).
TUNABLE_PARAMS = [name for g in PARAM_GROUPS.values() for name in g]

# Config params established at startup.
CONFIG_INT_PARAMS = {"DF_MC_DIR_EN": 0, "DF_YAW_HOLD_EN": 1, "DF_YAWSPD_PID_EN": 1}

# Params that PX4 stores as INT32 (encoded by bit-reinterpretation over MAVLink).
INT_PARAMS = set(CONFIG_INT_PARAMS.keys())

# Metric columns recorded in trials.csv (union across trial types; blank if N/A).
METRIC_KEYS = [
    "vx_p2p", "vy_p2p", "roll_p2p", "pitch_p2p", "yaw_p2p",
    "vx_rms", "vy_rms", "roll_rms", "pitch_rms", "yaw_rms",
    "roll_freq", "pitch_freq", "peak_att", "overshoot_pct", "settling_s",
]

# Per-param (min, max) clamps for the auto-tuner, from each param's metadata.
PARAM_BOUNDS = {
    "DF_YAWSPEED_P": (0.0, 5.0), "DF_YAWSPEED_I": (0.0, 1.0), "DF_YAWSPEED_D": (0.0, 2.0),
    "DF_YAW_FINE_P": (0.0, 5.0), "DF_YAW_FINE_I": (0.0, 1.0), "DF_YAW_FINE_D": (0.0, 5.0),
}

# Yaw auto-tune candidate moves, in priority order. Each move multiplies one gain
# (P/I by 1.3, D by 1.4) and is accepted only if the step response stays stable
# AND the cost improves. P first => the search pushes stiffness up first.
YT_MOVES = [
    ("DF_YAWSPEED_P", 1.3), ("DF_YAW_FINE_P", 1.3),
    ("DF_YAWSPEED_D", 1.4), ("DF_YAW_FINE_D", 1.4),
    ("DF_YAWSPEED_I", 1.3), ("DF_YAW_FINE_I", 1.3),
]

RAD2DEG = 57.29577951308232


def wrap180(deg):
    """Wrap an angle to (-180, 180]."""
    d = (deg + 180.0) % 360.0 - 180.0
    return d + 360.0 if d <= -180.0 else d


def wrap360(deg):
    """Wrap an angle to [0, 360)."""
    return deg % 360.0

# PX4 custom_mode main-mode field -> name.
PX4_MAIN_MODE = {
    1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO",
    5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE",
}
# PX4 custom main-mode value we command for Position control.
PX4_CUSTOM_MAIN_MODE_POSCTL = 3


def px4_mode_name(custom_mode):
    main = (int(custom_mode) >> 16) & 0xFF
    return PX4_MAIN_MODE.get(main, f"main{main}")


def decode_param(msg):
    """PX4 bit-reinterprets INT params into the float field. Decode by type."""
    ml = mavutil.mavlink
    int_types = {
        ml.MAV_PARAM_TYPE_UINT8: "<B", ml.MAV_PARAM_TYPE_INT8: "<b",
        ml.MAV_PARAM_TYPE_UINT16: "<H", ml.MAV_PARAM_TYPE_INT16: "<h",
        ml.MAV_PARAM_TYPE_UINT32: "<I", ml.MAV_PARAM_TYPE_INT32: "<i",
    }
    fmt = int_types.get(getattr(msg, "param_type", None))
    if fmt is None:
        return msg.param_value
    try:
        raw = struct.pack("<f", msg.param_value)
        sz = struct.calcsize(fmt)
        return struct.unpack(fmt, raw[:sz].ljust(sz, b"\x00"))[0]
    except Exception:
        return msg.param_value


# --------------------------------------------------------------------------- #
# Metric helpers (dependency-free; zero-crossing frequency estimate)
# --------------------------------------------------------------------------- #
def _p2p(vals):
    return (max(vals) - min(vals)) if vals else 0.0


def _rms(vals):
    """RMS about the mean (i.e. std dev) — the steady-state jitter measure."""
    n = len(vals)
    if n == 0:
        return 0.0
    m = sum(vals) / n
    return math.sqrt(sum((v - m) ** 2 for v in vals) / n)


def _zero_cross_freq(vals, duration_s):
    """Dominant oscillation frequency via mean-crossing count. No numpy/FFT."""
    n = len(vals)
    if n < 4 or duration_s <= 0:
        return 0.0
    m = sum(vals) / n
    dev = [v - m for v in vals]
    crossings = 0
    for i in range(1, n):
        if (dev[i - 1] <= 0.0 < dev[i]) or (dev[i - 1] >= 0.0 > dev[i]):
            crossings += 1
    # each full cycle has two mean-crossings
    return crossings / (2.0 * duration_s)


# --------------------------------------------------------------------------- #
# Raw-terminal single keypress reader
# --------------------------------------------------------------------------- #
class KeyReader:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.saved = None

    def cbreak(self):
        if self.saved is None:
            self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

    def restore(self):
        if self.saved is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def get(self, timeout):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            return sys.stdin.read(1)
        return None


# --------------------------------------------------------------------------- #
# Telemetry logger + capture buffer for metric computation
# --------------------------------------------------------------------------- #
# Downlink messages we need. ATTITUDE gives roll/pitch/yaw + body rates;
# LOCAL_POSITION_NED gives horizontal velocity used for translation metrics.
LOG_MESSAGES = {
    "ATTITUDE": 30,
    "LOCAL_POSITION_NED": 32,
}


def _flatten(d, max_list=16):
    out = {}
    for k, v in d.items():
        if k == "mavpackettype":
            continue
        if isinstance(v, (list, tuple)):
            for i, item in enumerate(v[:max_list]):
                out[f"{k}_{i}"] = item
        else:
            out[k] = v
    return out


class TelemetryLogger(threading.Thread):
    def __init__(self, master, run_dir):
        super().__init__(daemon=True)
        self.master = master
        self.run_dir = run_dir
        self._stop = threading.Event()
        self._writers = {}
        self.latest = {}
        self._latest_lock = threading.Lock()
        self.rx_count = 0
        self.last_rx_wall = 0.0

        # metric capture
        self._cap_lock = threading.Lock()
        self._capturing = False
        self._capture = []

        # PARAM_VALUE store — the logger is the SOLE reader of the link, so all
        # param reads/read-backs funnel through here (name -> (value, wall_time)).
        self._param_lock = threading.Lock()
        self._params = {}

    def get_param(self, name):
        with self._param_lock:
            return self._params.get(name)

    # ---- live snapshot -------------------------------------------------- #
    def get_latest(self):
        with self._latest_lock:
            return dict(self.latest)

    def _update_latest(self, mtype, d):
        with self._latest_lock:
            if mtype == "ATTITUDE":
                self.latest["att_roll"] = d.get("roll", 0.0) * RAD2DEG
                self.latest["att_pitch"] = d.get("pitch", 0.0) * RAD2DEG
                self.latest["att_yaw"] = d.get("yaw", 0.0) * RAD2DEG
                self.latest["rollspeed"] = d.get("rollspeed", 0.0) * RAD2DEG
                self.latest["pitchspeed"] = d.get("pitchspeed", 0.0) * RAD2DEG
                self.latest["yawspeed"] = d.get("yawspeed", 0.0) * RAD2DEG
            elif mtype == "LOCAL_POSITION_NED":
                self.latest["vx"] = d.get("vx", 0.0)
                self.latest["vy"] = d.get("vy", 0.0)
                self.latest["x"] = d.get("x", 0.0)
                self.latest["y"] = d.get("y", 0.0)
            elif mtype == "HEARTBEAT":
                self.latest["mode"] = px4_mode_name(d.get("custom_mode", 0))
                self.latest["fc_armed"] = bool(
                    d.get("base_mode", 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.rx_count += 1
            self.last_rx_wall = time.time()

    # ---- metric capture ------------------------------------------------- #
    def start_capture(self):
        with self._cap_lock:
            self._capture = []
            self._capturing = True

    def stop_capture(self):
        with self._cap_lock:
            self._capturing = False
            return list(self._capture)

    def _maybe_capture(self):
        """Append one merged sample (called on each ATTITUDE msg)."""
        with self._cap_lock:
            if not self._capturing:
                return
        with self._latest_lock:
            L = self.latest
            self._capture.append({
                "t": time.time(),
                "roll": L.get("att_roll", 0.0),
                "pitch": L.get("att_pitch", 0.0),
                "yaw": L.get("att_yaw", 0.0),
                "yawspeed": L.get("yawspeed", 0.0),
                "vx": L.get("vx", 0.0),
                "vy": L.get("vy", 0.0),
            })

    # ---- CSV ------------------------------------------------------------ #
    def _row(self, msgtype, data):
        flat = {"t_wall": time.time(), **_flatten(data)}
        if msgtype not in self._writers:
            path = os.path.join(self.run_dir, msgtype.lower() + ".csv")
            f = open(path, "w", newline="")
            cols = list(flat.keys())
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            self._writers[msgtype] = (f, w, cols)
        f, w, cols = self._writers[msgtype]
        w.writerow({k: flat.get(k, "") for k in cols})

    def run(self):
        want = set(LOG_MESSAGES.keys())
        while not self._stop.is_set():
            msg = self.master.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            mtype = msg.get_type()
            try:
                if mtype in want:
                    d = msg.to_dict()
                    self._row(mtype, d)
                    self._update_latest(mtype, d)
                    if mtype == "ATTITUDE":
                        self._maybe_capture()
                elif mtype == "HEARTBEAT":
                    self._update_latest(mtype, msg.to_dict())
                elif mtype == "PARAM_VALUE":
                    with self._param_lock:
                        self._params[msg.param_id.strip("\x00")] = (decode_param(msg), time.time())
            except Exception:
                pass

    def stop(self):
        self._stop.set()
        for f, _, _ in self._writers.values():
            try:
                f.flush()
                f.close()
            except Exception:
                pass


# --------------------------------------------------------------------------- #
# Tuner
# --------------------------------------------------------------------------- #
class Tuner:
    def __init__(self, args):
        self.args = args
        self.master = None
        self.target_system = 1
        self.target_component = 1

        self._send_lock = threading.Lock()
        self._target_lock = threading.Lock()
        # neutral throttle = hover/hold (POSCTL center stick). Horizontal drone
        # on a rope: vertical is carried by the wire, so center is the resting
        # command. Override with --hover-throttle if your airframe differs.
        self._neutral_thr = args.hover_throttle
        self._target = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0,
                        "throttle": self._neutral_thr}

        self.state = "IDLE"            # IDLE | IN_TRIAL
        self._state_lock = threading.Lock()
        self.armed = False
        self.arm_confirmed = False
        self.abort_evt = threading.Event()

        self.keys = KeyReader()
        self.run_dir = None
        self.logger = None
        self._injector = None
        self._stop_all = threading.Event()
        self._trial_thread = None

        # in-memory cache of tunable param values (name -> value)
        self.param_cache = {}
        self._trials_file = None
        self._trials_writer = None
        self.last_score = ""

        # HUD
        self.hud_enabled = not args.no_hud
        self._hud_pause = threading.Event()
        self._hud_thread = None
        self._out_lock = threading.Lock()
        self._rx_last_count = 0
        self._rx_last_t = time.time()
        self._rx_rate = 0.0

    # ---- low-level send ------------------------------------------------- #
    def _set_target(self, pitch=None, roll=None, yaw=None, throttle=None):
        with self._target_lock:
            if pitch is not None:
                self._target["pitch"] = pitch
            if roll is not None:
                self._target["roll"] = roll
            if yaw is not None:
                self._target["yaw"] = yaw
            if throttle is not None:
                self._target["throttle"] = throttle

    def _center(self):
        self._set_target(pitch=0.0, roll=0.0, yaw=0.0, throttle=self._neutral_thr)

    def _send_manual_control(self):
        with self._target_lock:
            t = dict(self._target)
        x = int(max(-1.0, min(1.0, t["pitch"])) * 1000)
        y = int(max(-1.0, min(1.0, t["roll"])) * 1000)
        r = int(max(-1.0, min(1.0, t["yaw"])) * 1000)
        z = int(max(0.0, min(1000.0, (t["throttle"] + 1.0) * 500.0)))
        with self._send_lock:
            self.master.mav.manual_control_send(self.target_system, x, y, z, r, 0)

    def _injector_loop(self):
        period = 1.0 / max(1, self.args.rate)
        last_hb = 0.0
        while not self._stop_all.is_set():
            try:
                self._send_manual_control()
                now = time.time()
                if now - last_hb > 1.0:
                    with self._send_lock:
                        self.master.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    last_hb = now
            except Exception:
                pass
            time.sleep(period)

    def _arm_cmd(self, arm, force=False):
        p2 = 21196.0 if force else 0.0
        with self._send_lock:
            self.master.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1.0 if arm else 0.0, p2, 0, 0, 0, 0, 0)

    # ---- connection ----------------------------------------------------- #
    def connect(self):
        is_serial = not self.args.connect.startswith(
            ("udp:", "udpin:", "udpout:", "tcp:"))
        print(f"Connecting to {self.args.connect}"
              f"{f' @ {self.args.baud} baud' if is_serial else ''} ...")
        if is_serial:
            self.master = mavutil.mavlink_connection(self.args.connect, baud=self.args.baud)
        else:
            self.master = mavutil.mavlink_connection(self.args.connect)
        self.master.wait_heartbeat(timeout=15)
        if self.master.target_system == 0:
            raise RuntimeError("No heartbeat / target system.")
        self.target_system = self.master.target_system
        self.target_component = self.master.target_component or 1
        print(f"Heartbeat from system {self.target_system} "
              f"component {self.target_component}")

    # ---- params: get / set with read-back ------------------------------ #
    def _request_param(self, name):
        with self._send_lock:
            self.master.mav.param_request_read_send(
                self.target_system, self.target_component,
                name.encode("ascii"), -1)

    def _get_param(self, name, timeout=3.0):
        """Read a param. Once the telemetry logger is running it is the SOLE
        reader of the link, so poll its PARAM_VALUE store (re-requesting on
        drop). Before the logger starts (startup), read the link directly."""
        t_req = time.time()
        self._request_param(name)
        use_logger = self.logger is not None and self.logger.is_alive()
        if use_logger:
            last_retry = t_req
            while time.time() - t_req < timeout:
                if self.abort_evt.is_set():
                    return None
                got = self.logger.get_param(name)
                if got is not None and got[1] >= t_req:
                    return got[0]
                if time.time() - last_retry > 0.6:     # reply likely dropped: re-request
                    self._request_param(name)
                    last_retry = time.time()
                time.sleep(0.03)
            return None
        # startup path: no logger thread yet, safe to read the link here
        while time.time() - t_req < timeout:
            msg = self.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
            if msg and msg.param_id.strip("\x00") == name:
                return decode_param(msg)
        return None

    def _set_param(self, name, value, timeout=3.0):
        """PARAM_SET + read-back verify. Returns (ok, readback)."""
        ml = mavutil.mavlink
        is_int = name in INT_PARAMS
        if is_int:
            # bit-reinterpret the int into the float field, matching PX4.
            enc = struct.unpack("<f", struct.pack("<i", int(round(value))))[0]
            ptype = ml.MAV_PARAM_TYPE_INT32
        else:
            enc = float(value)
            ptype = ml.MAV_PARAM_TYPE_REAL32
        with self._send_lock:
            self.master.mav.param_set_send(
                self.target_system, self.target_component,
                name.encode("ascii"), enc, ptype)
        rb = self._get_param(name, timeout=timeout)
        if rb is None:
            return False, None
        if is_int:
            ok = int(round(rb)) == int(round(value))
        else:
            ok = abs(float(rb) - float(value)) <= max(1e-4, abs(value) * 1e-3)
        if name in TUNABLE_PARAMS:
            self.param_cache[name] = rb
        return ok, rb

    def set_position_mode(self):
        """Command POSCTL via DO_SET_MODE (PX4 custom-main-mode convention)."""
        with self._send_lock:
            self.master.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                PX4_CUSTOM_MAIN_MODE_POSCTL, 0, 0, 0, 0, 0)
        time.sleep(0.5)
        hb = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
        return px4_mode_name(hb.custom_mode) if hb else "?"

    # ---- startup config + param cache ---------------------------------- #
    def setup_config(self):
        if self.args.no_setup:
            print("  (baseline config setup skipped: --no-setup)")
        else:
            print("Establishing baseline config (Direct off / Position / Yaw hold):")
            for name, val in CONFIG_INT_PARAMS.items():
                ok, rb = self._set_param(name, val)
                flag = "OK" if ok else "!! MISMATCH"
                print(f"  {name:<16}= {val}  -> readback {rb}  [{flag}]")
            mode = self.set_position_mode()
            flag = "OK" if mode == "POSCTL" else "!! not POSCTL (continuing)"
            print(f"  flight mode  -> {mode}  [{flag}]")

        # prime the tunable-param cache from the FC
        print("Reading current tunable params:")
        for name in TUNABLE_PARAMS:
            v = self._get_param(name)
            self.param_cache[name] = v
        self._print_params()
        pc = self.param_cache
        print("  Yaw PID @ start:  SPEED P/I/D = "
              f"{self._fmt(pc.get('DF_YAWSPEED_P'))}/{self._fmt(pc.get('DF_YAWSPEED_I'))}/"
              f"{self._fmt(pc.get('DF_YAWSPEED_D'))}   "
              f"FINE P/I/D = "
              f"{self._fmt(pc.get('DF_YAW_FINE_P'))}/{self._fmt(pc.get('DF_YAW_FINE_I'))}/"
              f"{self._fmt(pc.get('DF_YAW_FINE_D'))}")
        acc = self.param_cache.get("DF_ACC_PER_THR")
        if acc is not None and float(acc) < 1.0:
            print(f"  !! DF_ACC_PER_THR = {acc}: low for a big-fan drone. Its docs "
                  f"suggest ~3.0. This is the prime 'jumping' suspect — tune it first.")

    def _print_params(self):
        for g, names in PARAM_GROUPS.items():
            vals = "  ".join(f"{n}={self._fmt(self.param_cache.get(n))}" for n in names)
            print(f"  [{GROUP_TITLES[g]}] {vals}")

    @staticmethod
    def _fmt(v):
        if v is None:
            return "?"
        if isinstance(v, float):
            return f"{v:.4g}"
        return str(v)

    # ---- logging -------------------------------------------------------- #
    def start_logging(self, meta_extra):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "logs", f"run_{ts}")
        os.makedirs(self.run_dir, exist_ok=True)
        meta = {
            "timestamp": ts,
            "connect": self.args.connect,
            "rate_hz": self.args.rate,
            "nudge_amp": self.args.nudge_amp,
            "hold_secs": self.args.hold_secs,
            "hover_throttle": self.args.hover_throttle,
            "dry_run": self.args.dry_run,
            "params_at_start": {n: self.param_cache.get(n) for n in TUNABLE_PARAMS},
        }
        meta.update(meta_extra)
        with open(os.path.join(self.run_dir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2, default=str)

        self._trials_file = open(os.path.join(self.run_dir, "trials.csv"), "w", newline="")
        cols = ["t_wall", "trial", "direction"] + TUNABLE_PARAMS + METRIC_KEYS
        self._trials_writer = csv.DictWriter(self._trials_file, fieldnames=cols)
        self._trials_writer.writeheader()

        self.logger = TelemetryLogger(self.master, self.run_dir)
        self.logger.start()
        print(f"Logging to {self.run_dir}")

    def _record_trial(self, trial, direction, metrics):
        if not self._trials_writer:
            return
        row = {"t_wall": f"{time.time():.6f}", "trial": trial, "direction": direction}
        for n in TUNABLE_PARAMS:
            row[n] = self.param_cache.get(n, "")
        for k in METRIC_KEYS:
            row[k] = metrics.get(k, "")
        self._trials_writer.writerow(row)
        self._trials_file.flush()

    def request_streams(self):
        interval = int(1e6 / max(1, self.args.tele_rate))
        for mid in LOG_MESSAGES.values():
            with self._send_lock:
                self.master.mav.command_long_send(
                    self.target_system, self.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    mid, interval, 0, 0, 0, 0, 0)
            time.sleep(0.05)

    # ---- arm / disarm --------------------------------------------------- #
    def arm(self):
        if self.args.dry_run:
            self.log("[dry-run] arm skipped.")
            return
        if not self.arm_confirmed:
            self._hud_pause.set()
            self.keys.restore()
            try:
                print("\n*** SAFETY: props WILL spin. Confirm the drone is secured. ***")
                ans = input("Type ARM to confirm arming: ").strip()
            finally:
                self.keys.cbreak()
                self._hud_pause.clear()
            if ans.lower() != "arm":
                self.log("Arm cancelled.")
                return
            self.arm_confirmed = True
        self._center()
        self._arm_cmd(True)
        self.armed = True
        self.log("ARM command sent.")

    def disarm(self, force=False, reason=""):
        self._center()
        self._arm_cmd(False, force=force)
        if force:
            time.sleep(0.02)
            self._arm_cmd(False, force=True)
        self.armed = False
        tag = " (FORCED)" if force else ""
        self.log(f"DISARM sent{tag}. {reason}")

    # ---- trial helpers -------------------------------------------------- #
    def _sleep_abortable(self, dur):
        end = time.time() + dur
        while time.time() < end:
            if self.abort_evt.is_set():
                return False
            time.sleep(0.02)
        return True

    def _ramp_axis(self, axis, target, ramp_s=0.4):
        """Ramp one stick axis (pitch/roll/yaw) to target over ramp_s."""
        steps = max(1, int(ramp_s / 0.02))
        with self._target_lock:
            start = self._target[axis]
        for i in range(1, steps + 1):
            if self.abort_evt.is_set():
                return False
            f = i / steps
            self._set_target(**{axis: start + (target - start) * f})
            time.sleep(0.02)
        return True

    def _set_state(self, s):
        with self._state_lock:
            self.state = s

    def _get_state(self):
        with self._state_lock:
            return self.state

    # ---- trials --------------------------------------------------------- #
    def start_trial(self, kind, direction=""):
        if not self.armed and not self.args.dry_run:
            self.log("Not armed. Press 'a' to arm first.")
            return
        mode = self.logger.get_latest().get("mode") if self.logger else None
        if mode and mode != "POSCTL":
            self.log(f"!! MODE {mode}: expected POSCTL for position tuning "
                     f"(running anyway).")
        self.abort_evt.clear()
        self._set_state("IN_TRIAL")
        self._trial_thread = threading.Thread(
            target=self._run_trial, args=(kind, direction), daemon=True)
        self._trial_thread.start()

    def _run_trial(self, kind, direction):
        try:
            if kind == "hold":
                self._trial_hold()
            elif kind == "nudge":
                self._trial_nudge(direction)
            elif kind == "yaw":
                self._trial_yaw()
            elif kind == "yawtune":
                self._autotune_yaw()
        except KeyboardInterrupt:
            pass
        except Exception as e:
            self.log(f"trial error: {e}")
        finally:
            self._center()
            if self.abort_evt.is_set():
                self.log("=== trial ABORTED ===")
            self._set_state("IDLE")

    def _trial_hold(self):
        """Center sticks, measure steady-state jitter (the 'jumping' measure)."""
        dur = self.args.hold_secs
        self.log(f"=== HOLD/OBSERVE {dur:.1f}s (center sticks, measuring jitter) ===")
        self._center()
        if not self._sleep_abortable(0.8):   # let it settle before measuring
            return
        self.logger.start_capture()
        ok = self._sleep_abortable(dur)
        samples = self.logger.stop_capture()
        if not ok:
            return
        m = self._metrics_steady(samples, dur)
        self.last_score = self._score_hold(m)
        self.log("HOLD  " + self.last_score)
        self._record_trial("hold", "", m)

    def _trial_nudge(self, direction):
        """Small step on one axis, release, measure overshoot/settle/oscillation."""
        axis, sign = {"fwd": ("pitch", +1), "back": ("pitch", -1),
                      "right": ("roll", +1), "left": ("roll", -1)}[direction]
        amp = self.args.nudge_amp * sign
        push_s, settle_s = self.args.push_secs, self.args.settle_secs
        self.log(f"=== NUDGE {direction} (amp {self.args.nudge_amp:.2f}, "
                 f"push {push_s:.1f}s / settle {settle_s:.1f}s) ===")
        self._center()
        if not self._sleep_abortable(0.5):
            return
        # push phase (captured to get peak attitude)
        self.logger.start_capture()
        if not self._ramp_axis(axis, amp):
            self.logger.stop_capture()
            return
        if not self._sleep_abortable(push_s):
            self.logger.stop_capture()
            return
        push_samples = self.logger.stop_capture()
        # release phase (captured to get overshoot / oscillation / settling)
        self.logger.start_capture()
        if not self._ramp_axis(axis, 0.0, ramp_s=0.15):
            self.logger.stop_capture()
            return
        rel_start = time.time()
        ok = self._sleep_abortable(settle_s)
        rel_samples = self.logger.stop_capture()
        if not ok:
            return
        m = self._metrics_nudge(axis, sign, push_samples, rel_samples, rel_start, settle_s)
        self.last_score = self._score_nudge(m)
        self.log(f"NUDGE {direction}  " + self.last_score)
        self._record_trial("nudge", direction, m)

    def _trial_yaw(self):
        """Step DF_YAW_HOLD by a small angle, measure yaw settling."""
        step = self.args.yaw_step_deg
        cur = self.param_cache.get("DF_YAW_HOLD")
        if cur is None:
            cur = self._get_param("DF_YAW_HOLD") or 0.0
        target = (float(cur) + step) % 360.0
        self.log(f"=== YAW STEP {step:+.0f} deg  ({self._fmt(cur)} -> {target:.0f}) ===")
        ok, rb = self._set_param("DF_YAW_HOLD", target)
        if not ok:
            self.log(f"  !! could not set DF_YAW_HOLD (readback {rb})")
            return
        self.logger.start_capture()
        got = self._sleep_abortable(self.args.settle_secs)
        samples = self.logger.stop_capture()
        # restore original hold target
        self._set_param("DF_YAW_HOLD", float(cur))
        if not got:
            return
        m = self._metrics_yaw(samples, self.args.settle_secs)
        self.last_score = self._score_yaw(m)
        self.log("YAW   " + self.last_score)
        self._record_trial("yaw", f"{step:+.0f}deg", m)

    # ---- yaw auto-tune -------------------------------------------------- #
    def _yaw_step_measure(self, target):
        """Command DF_YAW_HOLD=target, wait for settle, and score the response.

        Returns dict with: settling_s, overshoot_pct, ss_error, osc_p2p,
        stable (bool), cost (float, inf if unstable)."""
        tol = 3.0          # deg: settled heading-error band
        rate_tol = 5.0     # deg/s: settled yaw-rate band
        settle_hold = 0.7  # s: must stay in-band this long to count as settled
        timeout = self.args.yt_settle_timeout

        lat = self.logger.get_latest()
        start = lat.get("att_yaw", 0.0)
        step_size = abs(wrap180(target - start))

        ok, rb = self._set_param("DF_YAW_HOLD", float(target))
        if not ok:
            self.log(f"  !! could not set DF_YAW_HOLD (readback {rb})")
            return None

        self.logger.start_capture()
        t0 = time.time()
        settled_s = None
        in_band_since = None
        while time.time() - t0 < timeout:
            if self.abort_evt.is_set():
                self.logger.stop_capture()
                return None
            lat = self.logger.get_latest()
            err = abs(wrap180(target - lat.get("att_yaw", 0.0)))
            yr = abs(lat.get("yawspeed", 0.0))
            if err < tol and yr < rate_tol:
                if in_band_since is None:
                    in_band_since = time.time()
                elif time.time() - in_band_since >= settle_hold:
                    settled_s = (in_band_since - t0)
                    break
            else:
                in_band_since = None
            time.sleep(0.05)
        samples = self.logger.stop_capture()

        if len(samples) < 4:
            return None
        yaws = [s["yaw"] for s in samples]
        errs = [abs(wrap180(target - y)) for y in yaws]
        # overshoot: how far past the target it travelled, in the travel direction
        direction = 1.0 if wrap180(target - start) >= 0 else -1.0
        beyond = max((direction * wrap180(y - target)) for y in yaws)
        overshoot_pct = (100.0 * max(0.0, beyond) / step_size) if step_size > 1.0 else 0.0
        # steady-state error over the last 1.0 s of samples
        t_end = samples[-1]["t"]
        tail = [e for e, s in zip(errs, samples) if s["t"] >= t_end - 1.0]
        ss_error = sum(tail) / len(tail) if tail else errs[-1]
        # residual oscillation: p2p of yaw over the last 1.5 s
        tail_yaw = [y for y, s in zip(yaws, samples) if s["t"] >= t_end - 1.5]
        osc_p2p = _p2p(tail_yaw)

        stable = (settled_s is not None
                  and overshoot_pct <= self.args.yt_overshoot_cap
                  and osc_p2p <= 4.0)
        settling_s = settled_s if settled_s is not None else timeout
        cost = (settling_s + 0.1 * overshoot_pct + 0.5 * ss_error) if stable else float("inf")
        return {"settling_s": settling_s, "overshoot_pct": overshoot_pct,
                "ss_error": ss_error, "osc_p2p": osc_p2p,
                "stable": stable, "cost": cost}

    def _autotune_yaw(self):
        """Hill-climb the 6 yaw gains toward a stiff, stable heading hold.

        Alternates the target between two headings 90 deg apart; each candidate
        gain raise is kept only if the step response stays stable AND cost
        improves, else reverted. best_stable is never left worse than the start.
        """
        # refresh working values from the FC
        params = list(PARAM_BOUNDS.keys())
        working = {}
        for n in params:
            v = self._get_param(n)
            if v is None:
                self.log(f"  !! could not read {n}; aborting auto-tune.")
                return
            working[n] = float(v)
        self._yt_start = dict(working)   # remember starting gains for the final diff

        base_hold = self.param_cache.get("DF_YAW_HOLD")
        if base_hold is None:
            base_hold = self._get_param("DF_YAW_HOLD") or self.logger.get_latest().get("att_yaw", 0.0)
        base_hold = wrap360(float(base_hold))
        targets = [base_hold, wrap360(base_hold + self.args.yt_step_deg)]

        self.log("=== YAW AUTO-TUNE (maximize stiffness, stay stable) ===")
        self.log("  start: " + "  ".join(f"{n}={working[n]:.3g}" for n in params))

        best = dict(working)
        # baseline step (idx 1) to establish current cost
        m = self._yaw_step_measure(targets[1])
        step_idx = 2
        if m is None:
            self.log("  baseline step failed/aborted; auto-tune stopped.")
            self._finish_autotune(best, params, base_hold)
            return
        best_cost = m["cost"]
        self._log_iter(0, "baseline", "-", working, m, accepted=True)
        self._append_autotune_row(0, "baseline", "-", working, m, True)
        if not m["stable"]:
            self.log("  !! baseline is UNSTABLE; will only try to improve, never degrade.")

        move_i = 0
        no_improve = 0
        it = 1
        while it <= self.args.yt_max_iters and not self.abort_evt.is_set():
            name, factor = YT_MOVES[move_i % len(YT_MOVES)]
            move_i += 1
            lo, hi = PARAM_BOUNDS[name]
            cur = working[name]
            new = min(hi, max(lo, cur * factor))
            if abs(new - cur) < 1e-6:            # already clamped: skip
                no_improve += 1
                if no_improve >= len(YT_MOVES):
                    self.log("  all candidate moves exhausted/clamped.")
                    break
                continue

            ok, rb = self._set_param(name, new)
            if not ok:
                self.log(f"  !! set {name} failed (readback {rb}); skipping.")
                continue
            working[name] = new
            target = targets[step_idx % 2]
            step_idx += 1
            m = self._yaw_step_measure(target)
            if m is None:
                # aborted or measurement failure: revert this move and stop
                self._set_param(name, cur)
                working[name] = cur
                break

            improved = m["stable"] and m["cost"] < best_cost - max(0.05, 0.02 * best_cost)
            if improved:
                best_cost = m["cost"]
                best = dict(working)
                no_improve = 0
                self._log_iter(it, name, f"{cur:.3g}->{new:.3g}", working, m, True)
                self._append_autotune_row(it, name, f"{cur:.3g}->{new:.3g}", working, m, True)
            else:
                self._set_param(name, cur)          # revert
                working[name] = cur
                no_improve += 1
                self._log_iter(it, name, f"{cur:.3g}->{new:.3g}", working, m, False)
                self._append_autotune_row(it, name, f"{cur:.3g}->{new:.3g}", working, m, False)
                if no_improve >= len(YT_MOVES):
                    self.log("  a full pass with no accepted move; converged.")
                    break
            it += 1

        self._finish_autotune(best, params, base_hold)

    def _finish_autotune(self, best, params, base_hold):
        """Apply best-stable gains, restore the hold target, report + log."""
        for n in params:
            self._set_param(n, best[n])
        self._set_param("DF_YAW_HOLD", float(base_hold))
        reason = "ABORTED" if self.abort_evt.is_set() else "done"
        start = getattr(self, "_yt_start", best)
        self.log(f"=== YAW AUTO-TUNE {reason}. Changes (before -> after): ===")
        n_changed = 0
        for n in params:
            b, a = start.get(n, best[n]), best[n]
            if abs(a - b) > 1e-6:
                pct = (100.0 * (a - b) / b) if abs(b) > 1e-9 else float("inf")
                arrow = "UP" if a > b else "DOWN"
                self.log(f"    {n:<16} {b:.4g} -> {a:.4g}  ({arrow} {pct:+.0f}%)")
                n_changed += 1
            else:
                self.log(f"    {n:<16} {a:.4g}  (unchanged)")
        self.log(f"  {n_changed} param(s) changed. Best-stable set is now LIVE on the FC.")
        self.log("  (press 'e' to export to tuned.params; persist via QGC to keep across reboot)")
        self.last_score = f"yaw auto-tune {reason}: {n_changed} gains changed"
        self._record_trial("yawtune", reason,
                           {"yaw_p2p": "", "yaw_rms": ""})

    def _log_iter(self, it, name, change, working, m, accepted):
        tag = "ACCEPT" if accepted else "reject"
        if m["stable"]:
            detail = (f"settle {m['settling_s']:.2f}s over {m['overshoot_pct']:.0f}% "
                      f"ss {m['ss_error']:.2f} deg cost {m['cost']:.2f}")
        else:
            detail = (f"UNSTABLE (over {m['overshoot_pct']:.0f}% osc {m['osc_p2p']:.1f} deg "
                      f"settle {m['settling_s']:.1f}s)")
        self.log(f"  [{it:2d}] {tag} {name} {change}: {detail}")

    def _append_autotune_row(self, it, name, change, working, m, accepted):
        if not self.run_dir:
            return
        path = os.path.join(self.run_dir, "yaw_autotune.csv")
        new_file = not os.path.exists(path)
        cols = (["t_wall", "iter", "move", "change", "accepted", "stable",
                 "settling_s", "overshoot_pct", "ss_error", "osc_p2p", "cost"]
                + list(PARAM_BOUNDS.keys()))
        try:
            with open(path, "a", newline="") as f:
                w = csv.DictWriter(f, fieldnames=cols)
                if new_file:
                    w.writeheader()
                row = {"t_wall": f"{time.time():.6f}", "iter": it, "move": name,
                       "change": change, "accepted": int(accepted),
                       "stable": int(m["stable"]), "settling_s": round(m["settling_s"], 3),
                       "overshoot_pct": round(m["overshoot_pct"], 2),
                       "ss_error": round(m["ss_error"], 3), "osc_p2p": round(m["osc_p2p"], 3),
                       "cost": (round(m["cost"], 3) if m["cost"] != float("inf") else "inf")}
                for n in PARAM_BOUNDS:
                    row[n] = round(working[n], 4)
                w.writerow(row)
        except Exception:
            pass

    # ---- metric computation -------------------------------------------- #
    @staticmethod
    def _series(samples, key):
        return [s[key] for s in samples]

    def _metrics_steady(self, samples, dur):
        if len(samples) < 4:
            return {}
        return {
            "vx_p2p": _p2p(self._series(samples, "vx")),
            "vy_p2p": _p2p(self._series(samples, "vy")),
            "roll_p2p": _p2p(self._series(samples, "roll")),
            "pitch_p2p": _p2p(self._series(samples, "pitch")),
            "yaw_p2p": _p2p(self._series(samples, "yaw")),
            "vx_rms": _rms(self._series(samples, "vx")),
            "vy_rms": _rms(self._series(samples, "vy")),
            "roll_rms": _rms(self._series(samples, "roll")),
            "pitch_rms": _rms(self._series(samples, "pitch")),
            "yaw_rms": _rms(self._series(samples, "yaw")),
            "roll_freq": _zero_cross_freq(self._series(samples, "roll"), dur),
            "pitch_freq": _zero_cross_freq(self._series(samples, "pitch"), dur),
        }

    def _metrics_nudge(self, axis, sign, push, rel, rel_start, settle_s):
        m = {}
        if len(push) >= 2:
            peak = max((sign * s[axis]) for s in push)  # in push direction
            m["peak_att"] = peak
        else:
            peak = 0.0
        if len(rel) >= 4:
            rel_axis = self._series(rel, axis)
            # overshoot = worst excursion opposite the push, vs push peak
            reverse = max((-sign * v) for v in rel_axis)
            m["overshoot_pct"] = (100.0 * reverse / peak) if peak > 1e-6 else 0.0
            m["roll_p2p" if axis == "roll" else "pitch_p2p"] = _p2p(rel_axis)
            freq = _zero_cross_freq(rel_axis, settle_s)
            m["roll_freq" if axis == "roll" else "pitch_freq"] = freq
            # settling: last time |axis| exceeded the band, relative to release
            band = self.args.settle_band_deg
            last_out = 0.0
            for s in rel:
                if abs(s[axis]) > band:
                    last_out = s["t"] - rel_start
            m["settling_s"] = last_out
            m["vx_p2p"] = _p2p(self._series(rel, "vx"))
            m["vy_p2p"] = _p2p(self._series(rel, "vy"))
        return m

    def _metrics_yaw(self, samples, dur):
        if len(samples) < 4:
            return {}
        yaw = self._series(samples, "yaw")
        return {
            "yaw_p2p": _p2p(yaw),
            "yaw_rms": _rms(yaw),
        }

    # ---- scorecards ----------------------------------------------------- #
    def _score_hold(self, m):
        if not m:
            return "(no telemetry captured)"
        return (f"jitter vRMS x{m['vx_rms']:.3f} y{m['vy_rms']:.3f} m/s | "
                f"att p2p R{m['roll_p2p']:.2f} P{m['pitch_p2p']:.2f} deg | "
                f"wobble R{m['roll_freq']:.2f} P{m['pitch_freq']:.2f} Hz")

    def _score_nudge(self, m):
        if not m:
            return "(no telemetry captured)"
        return (f"peak {m.get('peak_att', 0):.2f} deg | "
                f"overshoot {m.get('overshoot_pct', 0):.0f}% | "
                f"settle {m.get('settling_s', 0):.2f}s | "
                f"osc {max(m.get('roll_freq', 0), m.get('pitch_freq', 0)):.2f} Hz")

    def _score_yaw(self, m):
        if not m:
            return "(no telemetry captured)"
        return f"yaw p2p {m['yaw_p2p']:.2f} deg | yaw RMS {m['yaw_rms']:.2f} deg"

    def emergency_stop(self):
        self.abort_evt.set()
        self.disarm(force=True, reason="EMERGENCY (key during trial)")
        self._set_state("IDLE")

    # ---- param editor --------------------------------------------------- #
    def edit_param_group(self):
        if self.armed:
            self.log("Disarm first (press 'd') to edit parameters.")
            return
        self._hud_pause.set()
        self.keys.restore()
        try:
            print("\nParam groups:")
            gkeys = list(PARAM_GROUPS.keys())
            for i, g in enumerate(gkeys, 1):
                print(f"  {i}) {GROUP_TITLES[g]}")
            sel = input("Select group #: ").strip()
            if not sel.isdigit() or not (1 <= int(sel) <= len(gkeys)):
                print("  (cancelled)")
                return
            g = gkeys[int(sel) - 1]
            print(f"\n[{GROUP_TITLES[g]}]  (blank = keep)")
            for name in PARAM_GROUPS[g]:
                cur = self.param_cache.get(name)
                v = input(f"  {name} [{self._fmt(cur)}]: ").strip()
                if not v:
                    continue
                try:
                    val = float(v)
                except ValueError:
                    print(f"    (invalid number, skipped {name})")
                    continue
                ok, rb = self._set_param(name, val)
                flag = "OK" if ok else "!! MISMATCH"
                print(f"    {name} -> {self._fmt(rb)}  [{flag}]")
            print("\nActive params:")
            self._print_params()
        except Exception as e:
            print(f"  (edit error: {e})")
        finally:
            self.keys.cbreak()
            self._hud_pause.clear()

    def export_params(self):
        if not self.run_dir:
            self.log("No run dir; cannot export.")
            return
        # refresh cache from FC to export exactly what is live
        for name in TUNABLE_PARAMS:
            v = self._get_param(name)
            if v is not None:
                self.param_cache[name] = v
        path = os.path.join(self.run_dir, "tuned.params")
        try:
            with open(path, "w") as f:
                f.write("# SpinAir position-mode tuned params  "
                        f"{datetime.now().isoformat(timespec='seconds')}\n")
                for name in TUNABLE_PARAMS:
                    v = self.param_cache.get(name)
                    if v is not None:
                        f.write(f"{name},{v}\n")
            self.log(f"Exported tuned params -> {path}")
        except Exception as e:
            self.log(f"export error: {e}")

    # ---- HUD ------------------------------------------------------------ #
    def log(self, text):
        with self._out_lock:
            sys.stdout.write("\r\033[2K" + text + "\n")
            sys.stdout.flush()

    def _hud_line(self):
        with self._target_lock:
            t = dict(self._target)
        st = self._get_state()
        armed = "DRYRUN" if self.args.dry_run else ("ARMED" if self.armed else "DISARM")
        lat = self.logger.get_latest() if self.logger else {}
        if lat.get("fc_armed"):
            armed = "ARMED"
        elif self.args.dry_run:
            armed = "DRYRUN"
        mode = lat.get("mode", "?")
        mode_flag = "" if mode in ("POSCTL", "?") else "!"
        # only show sticks when off-center (i.e. during a trial) to keep the
        # idle line short enough to never wrap
        if max(abs(t["pitch"]), abs(t["roll"]), abs(t["yaw"])) > 0.005:
            sticks = f" | stk {t['pitch']:+.2f} {t['roll']:+.2f} {t['yaw']:+.2f}"
        else:
            sticks = ""
        if {"att_roll", "att_pitch", "att_yaw"} <= lat.keys():
            att = f"{lat['att_roll']:+.1f} {lat['att_pitch']:+.1f} {lat['att_yaw']:+.1f}"
        else:
            att = "-- -- --"
        if {"vx", "vy"} <= lat.keys():
            vel = f"{lat['vx']:+.2f} {lat['vy']:+.2f}"
        else:
            vel = "--"
        acc = self._fmt(self.param_cache.get("DF_ACC_PER_THR"))
        now = time.time()
        if self.logger:
            dc = self.logger.rx_count - self._rx_last_count
            dt = now - self._rx_last_t
            if dt >= 0.5:
                self._rx_rate = dc / dt
                self._rx_last_count = self.logger.rx_count
                self._rx_last_t = now
            age = now - self.logger.last_rx_wall if self.logger.last_rx_wall else 99
        else:
            age = 99
        link = f"rx{self._rx_rate:.0f}" + ("" if age < 1.0 else "!STALE")
        # compact: state, RPY (yaw is the key tuning signal), velocity, [sticks], acc, link
        return (f"[{armed} {mode}{mode_flag} {st}] RPY {att} | v {vel}"
                f"{sticks} | acc {acc} | {link}")

    def _hud_loop(self):
        import shutil
        last = None
        while not self._stop_all.is_set():
            if self.hud_enabled and not self._hud_pause.is_set():
                try:
                    line = self._hud_line().split("\n")[0]
                    # Truncate to terminal width so the line never wraps; a wrapped
                    # line breaks the in-place '\r' refresh and scrolls instead.
                    width = shutil.get_terminal_size((100, 20)).columns
                    if len(line) >= width:
                        line = line[:max(0, width - 1)]
                    if line != last:            # only redraw on change (less flicker)
                        with self._out_lock:
                            sys.stdout.write("\r\033[2K" + line)
                            sys.stdout.flush()
                        last = line
                except Exception:
                    pass
            time.sleep(0.2)

    # ---- main loop ------------------------------------------------------ #
    def print_help(self):
        self.log(
            "Keys (IDLE):  a=arm  d=disarm  p=edit param group (disarmed)\n"
            "  h=hold/observe   1=nudge fwd  2=back  3=right  4=left\n"
            "  y=single yaw step   t=AUTO-TUNE yaw PID (stiff hold)\n"
            "  e=export tuned params   ?=help   q=quit\n"
            "During a trial: ANY key = EMERGENCY force-disarm.\n"
            "Suggested order: DF_ACC_PER_THR first, then MPC_XY_VEL P->D->I, then 't' for yaw.")

    def loop(self):
        self.print_help()
        nudge_map = {"1": "fwd", "2": "back", "3": "right", "4": "left"}
        while not self._stop_all.is_set():
            key = self.keys.get(timeout=0.1)
            if key is None:
                continue
            if self._get_state() == "IN_TRIAL":
                self.emergency_stop()
                continue
            if key == "a":
                self.arm()
            elif key == "d":
                self.disarm()
            elif key == "p":
                self.edit_param_group()
            elif key == "h":
                self.start_trial("hold")
            elif key in nudge_map:
                self.start_trial("nudge", nudge_map[key])
            elif key == "y":
                self.start_trial("yaw")
            elif key == "t":
                self.start_trial("yawtune")
            elif key == "e":
                self.export_params()
            elif key == "?":
                self.print_help()
            elif key == "q":
                break

    # ---- lifecycle ------------------------------------------------------ #
    def run(self):
        self.keys.cbreak()
        try:
            self.connect()
            self.setup_config()
            self.start_logging({"mode_after_setup":
                                (self.logger.get_latest().get("mode") if self.logger else None)})
            self.request_streams()
            self._center()
            self._injector = threading.Thread(target=self._injector_loop, daemon=True)
            self._injector.start()
            time.sleep(0.3)
            if self.hud_enabled:
                self._hud_thread = threading.Thread(target=self._hud_loop, daemon=True)
                self._hud_thread.start()
            self.loop()
        finally:
            self.shutdown()

    def shutdown(self):
        try:
            self.abort_evt.set()
            if self.master is not None:
                try:
                    self.disarm(force=True, reason="shutdown")
                except Exception:
                    pass
            self._stop_all.set()
            time.sleep(0.2)
            if self.logger:
                self.logger.stop()
            if self._trials_file:
                try:
                    self._trials_file.close()
                except Exception:
                    pass
        finally:
            self.keys.restore()
            print("\nTerminal restored. Bye.")


def build_argparser():
    p = argparse.ArgumentParser(description="Position-mode PID tuner (SpinAir horizontal drone)")
    p.add_argument("--connect", default="udp:0.0.0.0:14551",
                   help="MAVLink endpoint: udp:127.0.0.1:14550 or serial /dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200,
                   help="serial baud (only for serial --connect)")
    p.add_argument("--no-setup", action="store_true", dest="no_setup",
                   help="skip auto config/mode setup (assume you already selected POSCTL)")
    p.add_argument("--rate", type=int, default=50, help="MANUAL_CONTROL uplink Hz")
    p.add_argument("--tele-rate", type=int, default=30, dest="tele_rate",
                   help="downlink telemetry Hz (ATTITUDE + LOCAL_POSITION_NED)")
    p.add_argument("--hover-throttle", type=float, default=0.0, dest="hover_throttle",
                   help="neutral throttle in POSCTL (-1..1); 0.0 = center/hold")
    p.add_argument("--nudge-amp", type=float, default=0.15, dest="nudge_amp",
                   help="stick step amplitude for nudge trials (0..1 fraction)")
    p.add_argument("--push-secs", type=float, default=1.0, dest="push_secs",
                   help="how long to hold the nudge before release")
    p.add_argument("--settle-secs", type=float, default=4.0, dest="settle_secs",
                   help="how long to observe after release / yaw step")
    p.add_argument("--settle-band-deg", type=float, default=1.0, dest="settle_band_deg",
                   help="attitude band (deg) that defines 'settled'")
    p.add_argument("--hold-secs", type=float, default=5.0, dest="hold_secs",
                   help="hold/observe trial duration")
    p.add_argument("--yaw-step-deg", type=float, default=15.0, dest="yaw_step_deg",
                   help="heading step for the single yaw trial ('y')")
    # yaw auto-tune ('t')
    p.add_argument("--yt-max-iters", type=int, default=14, dest="yt_max_iters",
                   help="yaw auto-tune: max candidate steps")
    p.add_argument("--yt-step-deg", type=float, default=90.0, dest="yt_step_deg",
                   help="yaw auto-tune: heading step between the two alternating targets")
    p.add_argument("--yt-overshoot-cap", type=float, default=25.0, dest="yt_overshoot_cap",
                   help="yaw auto-tune: overshoot %% above which a step is 'unstable'")
    p.add_argument("--yt-settle-timeout", type=float, default=12.0, dest="yt_settle_timeout",
                   help="yaw auto-tune: max seconds to wait for a step to settle")
    p.add_argument("--dry-run", action="store_true", help="do everything except arm")
    p.add_argument("--no-hud", action="store_true", help="disable the live status line")
    return p


def main():
    args = build_argparser().parse_args()
    if not sys.stdin.isatty():
        sys.exit("This tool needs an interactive terminal (TTY).")
    t = Tuner(args)

    def on_sigint(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_sigint)
    try:
        t.run()
    except KeyboardInterrupt:
        t.shutdown()


if __name__ == "__main__":
    main()
