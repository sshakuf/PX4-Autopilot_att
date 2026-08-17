#!/usr/bin/env python3
"""
Direct-Mode Actuator Movement Test
==================================

Interactive MAVLink harness to validate actuator response to forward/back/
left/right (+yaw) stick commands on the SpinAir horizontal drone while running
in ACRO + Direct Flight Control mode (DF_MC_DIR_EN=1).

In direct mode the sticks bypass all controllers and go straight to the control
allocator:  roll->torqueX, pitch->torqueY, yaw->torqueZ, throttle->thrustZ.
We inject MANUAL_CONTROL at ~50 Hz (exactly what direct mode consumes).

!!! SAFETY: this arms a real drone and spins the props. Secure the drone on its
    wire. Any key during a test = immediate force-disarm. Ctrl-C / crash / exit
    all force-disarm and restore the terminal.

See SPEC.md and README.md in this directory.
"""

import argparse
import json
import os
import select
import signal
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
# Parameters
# --------------------------------------------------------------------------- #
DIRECTIONS = ("fwd", "back", "right", "left", "yaw")


class Params:
    """Test parameters, overridable via CLI or the interactive `p` editor."""

    def __init__(self):
        self.amps = {d: [0.30, 0.40, 0.50] for d in DIRECTIONS}
        self.hold = 3.0          # steady-state hold per amplitude (s)
        self.center = 2.0        # centered pause between segments (s)
        self.ramp = 0.5          # linear ramp to target (s)
        self.base_throttle = 0.25  # collective held during a test (-1..1 -> 0..1 half)

    def as_dict(self):
        return {
            "amplitudes": {d: [round(a, 3) for a in self.amps[d]] for d in DIRECTIONS},
            "hold_s": self.hold,
            "center_s": self.center,
            "ramp_s": self.ramp,
            "base_throttle": self.base_throttle,
        }

    def summary(self):
        amp_str = "  ".join(
            f"{d}={','.join(str(int(round(a*100))) for a in self.amps[d])}"
            for d in DIRECTIONS
        )
        return (f"[params] {amp_str}\n"
                f"         hold={self.hold}s center={self.center}s "
                f"ramp={self.ramp}s base_throttle={self.base_throttle}")


def parse_amp_list(text):
    """'30,40,50' -> [0.3,0.4,0.5]  (percent -> fraction, clamped 0..1)."""
    out = []
    for tok in text.replace(" ", "").split(","):
        if not tok:
            continue
        v = float(tok) / 100.0
        out.append(max(0.0, min(1.0, v)))
    return out


# --------------------------------------------------------------------------- #
# Test definitions -> list of "moves":  (label, pitch, roll, yaw)
# throttle is held at base_throttle for the whole test by the runner.
# --------------------------------------------------------------------------- #
def _seg(label, pitch=0.0, roll=0.0, yaw=0.0):
    return (label, pitch, roll, yaw)


def _dir_move(direction, amp):
    if direction == "fwd":
        return _seg(f"fwd {int(amp*100)}%", pitch=+amp)
    if direction == "back":
        return _seg(f"back {int(amp*100)}%", pitch=-amp)
    if direction == "right":
        return _seg(f"right {int(amp*100)}%", roll=+amp)
    if direction == "left":
        return _seg(f"left {int(amp*100)}%", roll=-amp)
    if direction == "yaw":
        return _seg(f"yaw {int(amp*100)}%", yaw=+amp)
    raise ValueError(direction)


def build_test(key, p: Params):
    """Return (name, [moves]) for a test key, using current params."""
    def ramp(direction):
        return [_dir_move(direction, a) for a in p.amps[direction]]

    if key == "1":
        return "Forward ramp (pitch +)", ramp("fwd")
    if key == "2":
        return "Back ramp (pitch -)", ramp("back")
    if key == "3":
        return "Right ramp (roll +)", ramp("right")
    if key == "4":
        return "Left ramp (roll -)", ramp("left")
    if key == "5":
        moves = []
        for d in ("fwd", "back", "right", "left"):
            moves += ramp(d)
        return "Full matrix (fwd/back/right/left)", moves
    if key == "6":
        a = max(p.amps["right"] + p.amps["left"]) if (p.amps["right"] or p.amps["left"]) else 0.5
        return "Roll symmetry (right<->left @ max)", [
            _dir_move("right", a), _dir_move("left", a),
            _dir_move("right", a), _dir_move("left", a),
        ]
    if key == "7":
        a = max(p.amps["fwd"] + p.amps["back"]) if (p.amps["fwd"] or p.amps["back"]) else 0.5
        return "Pitch symmetry (fwd<->back @ max)", [
            _dir_move("fwd", a), _dir_move("back", a),
            _dir_move("fwd", a), _dir_move("back", a),
        ]
    if key == "8":
        return "Yaw ramp (yaw +)", ramp("yaw")
    if key == "9":
        return "Throttle-only baseline (no torque)", [_seg("hold base throttle")]
    if key == "0":
        a = min(min(p.amps[d]) for d in ("fwd", "back", "right", "left") if p.amps[d])
        return "Gentle all-directions @ min amp", [
            _dir_move("fwd", a), _dir_move("back", a),
            _dir_move("right", a), _dir_move("left", a),
        ]
    return None, None


# --------------------------------------------------------------------------- #
# Raw-terminal single keypress reader
# --------------------------------------------------------------------------- #
class KeyReader:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.saved = None  # captured lazily on first cbreak()

    def cbreak(self):
        if self.saved is None:
            self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

    def restore(self):
        if self.saved is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def get(self, timeout):
        """Return a single char, or None on timeout."""
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            return sys.stdin.read(1)
        return None


# --------------------------------------------------------------------------- #
# Telemetry logger
# --------------------------------------------------------------------------- #
# Downlink telemetry to stream. HIGHRES_IMU intentionally excluded: it is very
# high-rate and competes with the uplink MANUAL_CONTROL for link bandwidth
# (which was starving the FC's manual input to ~5 Hz). Keep this list lean.
LOG_MESSAGES = {
    "ATTITUDE": 30,
    "ATTITUDE_TARGET": 83,
    "ACTUATOR_OUTPUT_STATUS": 375,
    "SERVO_OUTPUT_RAW": 36,
    "ESC_STATUS": 291,
}

# PX4 custom_mode main-mode field -> name. Direct mode only engages in a manual
# mode (MANUAL / ACRO / STABILIZED); position/altitude modes bypass it.
PX4_MAIN_MODE = {
    1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO",
    5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE",
}
DIRECT_MODE_OK = {"MANUAL", "ACRO", "STABILIZED"}


def px4_mode_name(custom_mode):
    main = (int(custom_mode) >> 16) & 0xFF
    return PX4_MAIN_MODE.get(main, f"main{main}")


def _flatten(d, max_list=16):
    out = {}
    for k, v in d.items():
        if k in ("mavpackettype",):
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
        self._writers = {}   # msgtype -> (file, csv-columns)
        # live snapshot for the HUD
        self.latest = {}
        self._latest_lock = threading.Lock()
        self.rx_count = 0
        self.last_rx_wall = 0.0

    def get_latest(self):
        with self._latest_lock:
            return dict(self.latest)

    def _update_latest(self, mtype, d):
        with self._latest_lock:
            if mtype == "ATTITUDE":
                self.latest["att_roll"] = d.get("roll", 0.0) * 57.2958
                self.latest["att_pitch"] = d.get("pitch", 0.0) * 57.2958
                self.latest["att_yaw"] = d.get("yaw", 0.0) * 57.2958
            elif mtype == "SERVO_OUTPUT_RAW":
                self.latest["motors"] = [d.get(f"servo{i}_raw", 0) for i in range(1, 5)]
            elif mtype == "ACTUATOR_OUTPUT_STATUS":
                act = d.get("actuator", [])
                if isinstance(act, (list, tuple)) and "motors" not in self.latest:
                    self.latest["motors"] = list(act[:4])
            elif mtype == "HEARTBEAT":
                self.latest["mode"] = px4_mode_name(d.get("custom_mode", 0))
                self.latest["fc_armed"] = bool(
                    d.get("base_mode", 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.rx_count += 1
            self.last_rx_wall = time.time()

    def _row(self, msgtype, data):
        import csv
        flat = _flatten(data)
        flat = {"t_wall": time.time(), **flat}
        if msgtype not in self._writers:
            path = os.path.join(self.run_dir, msgtype.lower() + ".csv")
            f = open(path, "w", newline="")
            cols = list(flat.keys())
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            self._writers[msgtype] = (f, w, cols)
        f, w, cols = self._writers[msgtype]
        # keep to known columns (ignore any new fields)
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
                elif mtype == "HEARTBEAT":
                    # tracked for live mode/armed state, not written to CSV
                    self._update_latest(mtype, msg.to_dict())
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
# Main harness
# --------------------------------------------------------------------------- #
class Harness:
    def __init__(self, args):
        self.args = args
        self.params = Params()
        self._apply_cli_params()

        self.master = None
        self.target_system = 1
        self.target_component = 1

        self._send_lock = threading.Lock()
        self._target_lock = threading.Lock()
        # shared stick target streamed by the injector
        self._target = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0, "throttle": -1.0}

        self.state = "IDLE"            # IDLE | IN_TEST
        self._state_lock = threading.Lock()
        self.armed = False
        self.arm_confirmed = False     # typed ARM once per session
        self.abort_evt = threading.Event()

        self.keys = KeyReader()
        self.run_dir = None
        self.events_file = None
        self.logger = None
        self._injector = None
        self._stop_all = threading.Event()
        self._test_thread = None

        # live HUD
        self.hud_enabled = not args.no_hud
        self._hud_pause = threading.Event()   # set => HUD stops drawing (cooked input)
        self._hud_thread = None
        self._out_lock = threading.Lock()
        self._rx_last_count = 0
        self._rx_last_t = time.time()
        self._rx_rate = 0.0

        # TX self-check: record our actual MANUAL_CONTROL send intervals
        self._tx_intervals = []      # ms between consecutive sends
        self._tx_last_send = None
        self._tx_count = 0
        self._tx_rate_last_count = 0
        self._tx_rate_last_t = time.time()
        self._tx_rate = 0.0

    # ---- params -------------------------------------------------------- #
    def _apply_cli_params(self):
        a = self.args
        if a.amplitudes:
            g = parse_amp_list(a.amplitudes)
            for d in DIRECTIONS:
                self.params.amps[d] = list(g)
        for d, val in (("fwd", a.fwd_amp), ("back", a.back_amp),
                       ("right", a.right_amp), ("left", a.left_amp),
                       ("yaw", a.yaw_amp)):
            if val:
                self.params.amps[d] = parse_amp_list(val)
        if a.hold is not None:
            self.params.hold = a.hold
        if a.center is not None:
            self.params.center = a.center
        if a.ramp is not None:
            self.params.ramp = a.ramp
        if a.base_throttle is not None:
            self.params.base_throttle = max(-1.0, min(1.0, a.base_throttle))

    # ---- low-level send ------------------------------------------------ #
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

    def _center(self, throttle=None):
        self._set_target(pitch=0.0, roll=0.0, yaw=0.0, throttle=throttle)

    def _send_manual_control(self):
        with self._target_lock:
            t = dict(self._target)
        x = int(max(-1.0, min(1.0, t["pitch"])) * 1000)
        y = int(max(-1.0, min(1.0, t["roll"])) * 1000)
        r = int(max(-1.0, min(1.0, t["yaw"])) * 1000)
        z = int(max(0.0, min(1000.0, (t["throttle"] + 1.0) * 500.0)))
        with self._send_lock:
            self.master.mav.manual_control_send(
                self.target_system, x, y, z, r, 0)

    def _injector_loop(self):
        period = 1.0 / max(1, self.args.rate)
        last_hb = 0.0
        while not self._stop_all.is_set():
            try:
                self._send_manual_control()
                now = time.time()
                # TX self-check: measure our real send cadence
                if self._tx_last_send is not None:
                    self._tx_intervals.append((now - self._tx_last_send) * 1000.0)
                self._tx_last_send = now
                self._tx_count += 1
                if now - self._tx_rate_last_t >= 0.5:
                    self._tx_rate = (self._tx_count - self._tx_rate_last_count) / (now - self._tx_rate_last_t)
                    self._tx_rate_last_count = self._tx_count
                    self._tx_rate_last_t = now
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

    # ---- connection & checks ------------------------------------------ #
    def connect(self):
        # baud is used only for serial devices (e.g. /dev/ttyACM0); ignored for udp/tcp.
        is_serial = not self.args.connect.startswith(("udp:", "udpin:", "udpout:", "tcp:"))
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

    @staticmethod
    def _decode_param(msg):
        """PX4 encodes INT params by bit-reinterpreting them into the float
        field. Decode according to param_type so INT32 params (e.g.
        DF_MC_DIR_EN, COM_RC_IN_MODE) read as their true integer value."""
        import struct
        ml = mavutil.mavlink
        int_types = {
            ml.MAV_PARAM_TYPE_UINT8: "<B", ml.MAV_PARAM_TYPE_INT8: "<b",
            ml.MAV_PARAM_TYPE_UINT16: "<H", ml.MAV_PARAM_TYPE_INT16: "<h",
            ml.MAV_PARAM_TYPE_UINT32: "<I", ml.MAV_PARAM_TYPE_INT32: "<i",
        }
        fmt = int_types.get(getattr(msg, "param_type", None))
        if fmt is None:
            return msg.param_value  # REAL32 etc. -> use as-is
        try:
            raw = struct.pack("<f", msg.param_value)
            return struct.unpack(fmt, raw[:struct.calcsize(fmt)].ljust(4, b"\x00")[:struct.calcsize(fmt)])[0]
        except Exception:
            return msg.param_value

    def _get_param(self, name, timeout=3.0):
        with self._send_lock:
            self.master.mav.param_request_read_send(
                self.target_system, self.target_component,
                name.encode("ascii"), -1)
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
            if msg and msg.param_id.strip("\x00") == name:
                return self._decode_param(msg)
        return None

    def preflight(self):
        info = {}
        if self.args.skip_checks:
            print("  (preflight param checks skipped: --skip-checks)")
            return info
        dir_en = self._get_param("DF_MC_DIR_EN")
        rcmode = self._get_param("COM_RC_IN_MODE")
        info["DF_MC_DIR_EN"] = dir_en
        info["COM_RC_IN_MODE"] = rcmode
        for k in ("DF_MC_DIR_RP", "DF_MC_DIR_YAW", "DF_MC_DIR_THR"):
            info[k] = self._get_param(k)

        print(f"  DF_MC_DIR_EN   = {dir_en}")
        print(f"  COM_RC_IN_MODE = {rcmode}")
        print(f"  DF_MC_DIR_RP/YAW/THR = "
              f"{info.get('DF_MC_DIR_RP')}/{info.get('DF_MC_DIR_YAW')}/{info.get('DF_MC_DIR_THR')}")

        # current flight mode (direct mode only engages in a manual mode)
        hb = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
        mode = px4_mode_name(hb.custom_mode) if hb else "?"
        info["flight_mode"] = mode
        print(f"  FLIGHT MODE    = {mode}")

        # Warn-only: never abort, just surface anything that looks off.
        if dir_en is not None and int(dir_en) != 1:
            print("  !! DF_MC_DIR_EN != 1 : direct mode appears OFF (continuing anyway).")
        if rcmode is not None and int(rcmode) not in (1, 2, 3):
            print("  !! COM_RC_IN_MODE not in (1,2,3): MAVLink manual control "
                  "may be ignored (continuing anyway).")
        if mode not in DIRECT_MODE_OK and mode != "?":
            print(f"  !! MODE {mode}: direct mode only engages in MANUAL/ACRO/STABILIZED.")
            print("     In this mode the normal controllers run and the actuator")
            print("     response will NOT be the clean open-loop direct-mode output.")
        return info

    # ---- logging ------------------------------------------------------- #
    def start_logging(self, meta_extra):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "logs", f"run_{ts}")
        os.makedirs(self.run_dir, exist_ok=True)
        meta = {
            "timestamp": ts,
            "connect": self.args.connect,
            "params": self.params.as_dict(),
            "rate_hz": self.args.rate,
            "dry_run": self.args.dry_run,
        }
        meta.update(meta_extra)
        with open(os.path.join(self.run_dir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2, default=str)

        import csv
        self.events_file = open(os.path.join(self.run_dir, "events.csv"), "w", newline="")
        self._events = csv.writer(self.events_file)
        self._events.writerow(["t_wall", "phase", "test", "direction",
                               "amplitude", "pitch", "roll", "yaw", "throttle"])
        self.logger = TelemetryLogger(self.master, self.run_dir)
        self.logger.start()
        print(f"Logging to {self.run_dir}")

    def _event(self, phase, test="", direction="", amplitude=""):
        with self._target_lock:
            t = dict(self._target)
        if self._events:
            self._events.writerow([f"{time.time():.6f}", phase, test, direction,
                                   amplitude, t["pitch"], t["roll"], t["yaw"], t["throttle"]])
            self.events_file.flush()

    def request_streams(self):
        # Keep downlink telemetry modest so it doesn't starve the uplink
        # MANUAL_CONTROL (see --tele-rate, default 20 Hz).
        interval = int(1e6 / max(1, self.args.tele_rate))
        for mid in LOG_MESSAGES.values():
            with self._send_lock:
                self.master.mav.command_long_send(
                    self.target_system, self.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    mid, interval, 0, 0, 0, 0, 0)
            time.sleep(0.05)

    # ---- arm / disarm -------------------------------------------------- #
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
        # ensure safe stream before arming
        self._center(throttle=-1.0)
        self._arm_cmd(True)
        self.armed = True
        self.log("ARM command sent. Motors at idle throttle.")

    def disarm(self, force=False, reason=""):
        self._center(throttle=-1.0)
        self._arm_cmd(False, force=force)
        if force:
            # fire twice for good measure on an emergency
            time.sleep(0.02)
            self._arm_cmd(False, force=True)
        self.armed = False
        tag = " (FORCED)" if force else ""
        self.log(f"DISARM sent{tag}. {reason}")

    # ---- test execution ----------------------------------------------- #
    def _sleep_abortable(self, dur):
        end = time.time() + dur
        while time.time() < end:
            if self.abort_evt.is_set():
                return False
            time.sleep(0.02)
        return True

    def _ramp_to(self, pitch, roll, yaw):
        steps = max(1, int(self.params.ramp / 0.02))
        with self._target_lock:
            p0, r0, y0 = self._target["pitch"], self._target["roll"], self._target["yaw"]
        for i in range(1, steps + 1):
            if self.abort_evt.is_set():
                return False
            f = i / steps
            self._set_target(pitch=p0 + (pitch - p0) * f,
                             roll=r0 + (roll - r0) * f,
                             yaw=y0 + (yaw - y0) * f)
            time.sleep(0.02)
        return True

    def _run_test(self, key):
        name, moves = build_test(key, self.params)
        if not moves:
            self.log(f"No test mapped to '{key}'.")
            self._set_state("IDLE")
            return
        self.log(f"=== TEST {key}: {name} ===")
        self.log(self.params.summary())
        self._event("test_start", test=key)
        base = self.params.base_throttle
        try:
            # ramp throttle up to base at center
            self._center()
            self._ramp_to(0.0, 0.0, 0.0)
            self._set_target(throttle=base)
            if not self._sleep_abortable(self.params.center):
                raise KeyboardInterrupt
            for (label, pitch, roll, yaw) in moves:
                if self.abort_evt.is_set():
                    break
                # center
                self._center(throttle=base)
                self._event("center", test=key)
                if not self._sleep_abortable(self.params.center):
                    break
                # ramp to target + hold
                self.log(f"  -> {label}")
                if not self._ramp_to(pitch, roll, yaw):
                    break
                self._event("hold", test=key, direction=label,
                            amplitude=max(abs(pitch), abs(roll), abs(yaw)))
                if not self._sleep_abortable(self.params.hold):
                    break
            # return to center + idle throttle
            self._center(throttle=-1.0)
            self._event("test_end", test=key)
        except KeyboardInterrupt:
            pass
        finally:
            self._center(throttle=-1.0)
            if self.abort_evt.is_set():
                self.log(f"=== TEST {key} ABORTED ===")
            else:
                self.log(f"=== TEST {key} complete ===")
            self._set_state("IDLE")

    def _set_state(self, s):
        with self._state_lock:
            self.state = s

    def _get_state(self):
        with self._state_lock:
            return self.state

    def start_test(self, key):
        if not self.armed and not self.args.dry_run:
            self.log("Not armed. Press 'a' to arm first.")
            return
        mode = self.logger.get_latest().get("mode") if self.logger else None
        if mode and mode not in DIRECT_MODE_OK:
            self.log(f"!! MODE {mode}: direct mode NOT engaged — response will be "
                     f"closed-loop, not clean. Switch to ACRO/MANUAL. (running anyway)")
        self.abort_evt.clear()
        self._set_state("IN_TEST")
        self._test_thread = threading.Thread(target=self._run_test, args=(key,), daemon=True)
        self._test_thread.start()

    def emergency_stop(self):
        self.abort_evt.set()
        self.disarm(force=True, reason="EMERGENCY (key during test)")
        self._set_state("IDLE")

    # ---- interactive param editor ------------------------------------- #
    def edit_params(self):
        if self.armed:
            self.log("Disarm first (press 'd') to edit parameters.")
            return
        self._hud_pause.set()
        self.keys.restore()
        try:
            print("\n" + self.params.summary())
            print("Edit fields (blank = keep). Amplitudes as percent list e.g. 30,40,50")
            for d in DIRECTIONS:
                cur = ",".join(str(int(round(a*100))) for a in self.params.amps[d])
                v = input(f"  {d} amp% [{cur}]: ").strip()
                if v:
                    self.params.amps[d] = parse_amp_list(v)
            for field in ("hold", "center", "ramp", "base_throttle"):
                cur = getattr(self.params, field)
                v = input(f"  {field} [{cur}]: ").strip()
                if v:
                    setattr(self.params, field, float(v))
            print(self.params.summary())
        except Exception as e:
            print(f"  (edit error: {e})")
        finally:
            self.keys.cbreak()
            self._hud_pause.clear()

    # ---- live HUD ------------------------------------------------------ #
    def log(self, text):
        """Print a message without clobbering the in-place HUD line."""
        with self._out_lock:
            sys.stdout.write("\r\033[2K" + text + "\n")
            sys.stdout.flush()

    def _hud_line(self):
        with self._target_lock:
            t = dict(self._target)
        st = self._get_state()
        armed = "ARMED" if self.armed else "DISARM"
        if self.args.dry_run:
            armed = "DRYRUN"
        z = int(max(0.0, min(1000.0, (t["throttle"] + 1.0) * 500.0)))
        sent = (f"p{t['pitch']:+.2f} r{t['roll']:+.2f} "
                f"y{t['yaw']:+.2f} thr{t['throttle']:+.2f}(z{z})")
        lat = self.logger.get_latest() if self.logger else {}
        mode = lat.get("mode", "?")
        mode_flag = "" if mode in DIRECT_MODE_OK or mode == "?" else "!"
        if lat.get("fc_armed"):
            armed = "ARMED"
        elif self.args.dry_run:
            armed = "DRYRUN"
        if {"att_roll", "att_pitch", "att_yaw"} <= lat.keys():
            att = (f"R{lat['att_roll']:+5.1f} P{lat['att_pitch']:+5.1f} "
                   f"Y{lat['att_yaw']:+6.1f}")
        else:
            att = "R  --  P  --  Y  --"
        motors = lat.get("motors")
        mstr = "[" + " ".join(f"{int(m):4d}" for m in motors) + "]" if motors else "[   no motor tlm  ]"
        # rx rate
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
        link = f"tx{self._tx_rate:3.0f} rx{self._rx_rate:3.0f}Hz" + ("" if age < 1.0 else " STALE")
        return (f"[{armed}|{mode}{mode_flag}|{st}] SENT {sent} | ATT {att} | "
                f"M{mstr} | {link}")

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

    # ---- main loop ----------------------------------------------------- #
    def print_help(self):
        self.log(
            "Keys (IDLE):  a=arm  d=disarm  p=edit params (disarmed)  "
            "1-9,0=run test  h=help  q=quit\n"
            "Tests: 1 fwd  2 back  3 right  4 left  5 full-matrix  6 roll-sym  "
            "7 pitch-sym  8 yaw  9 throttle-only  0 gentle-all\n"
            "During a test: ANY key = EMERGENCY force-disarm.")

    def loop(self):
        self.print_help()
        while not self._stop_all.is_set():
            key = self.keys.get(timeout=0.1)
            if key is None:
                continue
            if self._get_state() == "IN_TEST":
                # any key = emergency stop
                self.emergency_stop()
                continue
            # IDLE
            if key == "a":
                self.arm()
            elif key == "d":
                self.disarm()
            elif key == "p":
                self.edit_params()
            elif key == "h":
                self.print_help()
            elif key == "q":
                break
            elif key in "1234567890":
                self.start_test(key)
            else:
                pass

    # ---- lifecycle ----------------------------------------------------- #
    def run(self):
        self.keys.cbreak()
        try:
            self.connect()
            info = self.preflight()
            self.start_logging(info)
            self.request_streams()
            # start injector (streams center + idle before any arm)
            self._center(throttle=-1.0)
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
            # force disarm on the way out
            if self.master is not None:
                try:
                    self.disarm(force=True, reason="shutdown")
                except Exception:
                    pass
            self._stop_all.set()
            time.sleep(0.2)
            if self.logger:
                self.logger.stop()
            if self.events_file:
                self.events_file.close()
            self._write_tx_timing()
        finally:
            self.keys.restore()
            print("\nTerminal restored. Bye.")

    def _write_tx_timing(self):
        """Persist our actual MANUAL_CONTROL send cadence for review."""
        if not self.run_dir or not self._tx_intervals:
            return
        import csv, statistics as st
        iv = self._tx_intervals
        path = os.path.join(self.run_dir, "tx_timing.csv")
        try:
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["send_index", "interval_ms"])
                for i, v in enumerate(iv):
                    w.writerow([i, round(v, 2)])
            over = sum(1 for v in iv if v > 100)
            print(f"TX self-check: {len(iv)+1} sends, "
                  f"median {st.median(iv):.0f} ms, max {max(iv):.0f} ms, "
                  f"{over} gaps >100 ms  (target: 50 Hz / 20 ms, 0 gaps).")
            print(f"  -> if our TX is clean but FC saw ~5 Hz, the bottleneck is the LINK.")
        except Exception:
            pass


def build_argparser():
    p = argparse.ArgumentParser(description="Direct-mode actuator movement test")
    p.add_argument("--connect", default="udp:0.0.0.0:14551",
                   help="MAVLink endpoint: udp:127.0.0.1:14550 (MAVProxy localhost) "
                        "or a serial device like /dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200,
                   help="serial baud (only for serial --connect; ignored for udp)")
    p.add_argument("--amplitudes", help="global default %% list, e.g. 30,40,50")
    p.add_argument("--fwd-amp")
    p.add_argument("--back-amp")
    p.add_argument("--right-amp")
    p.add_argument("--left-amp")
    p.add_argument("--yaw-amp")
    p.add_argument("--hold", type=float)
    p.add_argument("--center", type=float)
    p.add_argument("--ramp", type=float)
    p.add_argument("--base-throttle", type=float, dest="base_throttle")
    p.add_argument("--rate", type=int, default=50, help="MANUAL_CONTROL uplink Hz")
    p.add_argument("--tele-rate", type=int, default=20, dest="tele_rate",
                   help="downlink telemetry Hz (kept low to protect uplink bandwidth)")
    p.add_argument("--dry-run", action="store_true", help="do everything except arm")
    p.add_argument("--skip-checks", action="store_true")
    p.add_argument("--no-hud", action="store_true", help="disable the live status line")
    return p


def main():
    args = build_argparser().parse_args()
    if not sys.stdin.isatty():
        sys.exit("This tool needs an interactive terminal (TTY).")
    h = Harness(args)

    def on_sigint(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_sigint)
    try:
        h.run()
    except KeyboardInterrupt:
        h.shutdown()


if __name__ == "__main__":
    main()
