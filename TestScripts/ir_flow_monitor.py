#!/usr/bin/env python3
"""
Live body-frame view of the IR beacon target and the optical-flow / EKF motion.

Screen layout (top-down, body frame):
    UP    = drone nose      (body +X forward)
    RIGHT = drone right     (body +Y)
    centre = camera boresight

  * red dot          -> IR beacon position, projected to the ground using
                        angle_x/angle_y and the current height
  * dashed rectangle -> camera ground footprint (where the beacon can still
                        be seen at this height). Beacon outside it = no data.
  * drifting dots    -> apparent ground motion, i.e. what the optical flow
                        sees. Drone moves forward => dots move DOWN.
  * bold arrow       -> drone velocity in body frame (EKF, flow-driven)

Why pymavlink and not MAVSDK: ir_camera_report is a custom uORB topic, not a
MAVLink message. MAVSDK only exposes MAVLink and MAVSDK-Python has no
passthrough plugin, so it cannot see it at all. We instead drive the NuttX
shell over MAVLink SERIAL_CONTROL (what QGC's MAVLink Console does) and parse
`listener ir_camera_report`. Motion comes from ordinary MAVLink telemetry.

IMPORTANT: the shell is exclusive. Close QGroundControl before running, or it
will fight this script for the console.

Usage:
    python3 ir_flow_monitor.py                       # auto-detect USB
    python3 ir_flow_monitor.py --connect /dev/cu.usbmodem01 --csv run1.csv
    python3 ir_flow_monitor.py --connect udp:127.0.0.1:14550

Keys:
    r  reset displacement origin (use at the start of a hand-carry test)
    c  clear beacon trail
    +/- change dot-drift gain (1.0 = true scale)
    q  quit
"""

import argparse
import csv
import glob
import math
import os
import queue
import re
import sys
import threading
import time
from collections import deque

# DEBUG_FLOAT_ARRAY is message id 350, i.e. MAVLink 2 only. pymavlink defaults
# to the v1.0 ardupilotmega dialect, where that message does not exist at all --
# so this must be set BEFORE importing pymavlink.
os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "common")

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink missing:  pip install -r requirements.txt")

if not hasattr(mavutil.mavlink, "MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY"):
    sys.exit("pymavlink loaded a MAVLink 1 dialect without DEBUG_FLOAT_ARRAY.\n"
             "Run with:  MAVLINK20=1 MAVLINK_DIALECT=common python3 ir_flow_monitor.py")

try:
    import numpy as np
    import matplotlib
    import matplotlib.animation
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from matplotlib.widgets import Button
except ImportError:
    sys.exit("matplotlib/numpy missing:  pip install -r requirements.txt")


# ----------------------------------------------------------------------------
# shared state
# ----------------------------------------------------------------------------

class State:
    """Everything the MAVLink thread produces and the UI thread consumes."""

    def __init__(self):
        self.lock = threading.Lock()
        # IR
        self.ir_t = 0.0            # local monotonic time of last IR sample
        self.dx_px = 0.0
        self.dy_px = 0.0
        self.angle_x = float("nan")
        self.angle_y = float("nan")
        self.valid = False
        self.spot_id = 0
        self.spot_score = 0
        self.frame_w = 1236
        self.frame_h = 960
        self.ir_count = 0
        self.ir_via = "-"          # "DEBUG_FLOAT_ARRAY" or "shell"
        # motion
        self.att_t = 0.0
        self.yaw = 0.0             # rad
        self.roll = 0.0
        self.pitch = 0.0
        self.yawspeed = 0.0        # rad/s -- needed to spot rotation-induced flow
        self.rollspeed = 0.0
        self.pitchspeed = 0.0
        self.lp_t = 0.0
        self.x = 0.0               # EKF NED position
        self.y = 0.0
        self.vn = 0.0
        self.ve = 0.0
        self.height = float("nan")  # AGL, m
        # arming (authoritative: HEARTBEAT base_mode, never our own guess)
        self.armed = False
        self.hb_t = 0.0
        self.ack = ""              # last COMMAND_ACK, human readable
        self.ack_t = 0.0
        self.tgt_sys = 0
        self.tgt_comp = 1
        # bookkeeping
        self.origin = None         # (x, y) captured on 'r'
        self.connected = False
        self.shell_lines = 0
        self.last_err = ""


STATE = State()
STOP = threading.Event()
# UI thread -> MAVLink thread. pymavlink connections are not safe for concurrent
# sends, so the UI never touches the link; it just posts a request here.
CMD_Q = queue.Queue()

ACK_RESULT = {
    0: "ACCEPTED", 1: "TEMPORARILY REJECTED", 2: "DENIED", 3: "UNSUPPORTED",
    4: "FAILED", 5: "IN PROGRESS", 6: "CANCELLED",
}


# ----------------------------------------------------------------------------
# MAVLink / shell plumbing
# ----------------------------------------------------------------------------

def autodetect_port():
    """Pick the most likely PX4 USB CDC-ACM device for this OS."""
    pats = ["/dev/cu.usbmodem*", "/dev/tty.usbmodem*",  # macOS
            "/dev/serial/by-id/*PX4*", "/dev/ttyACM*"]   # Linux
    for p in pats:
        hits = sorted(glob.glob(p))
        if hits:
            return hits[0]
    return None


class ShellReader:
    """Runs `listener ir_camera_report` on the FC's nsh over SERIAL_CONTROL.

    PX4 pushes console output back as SERIAL_CONTROL messages once we've sent a
    command with RESPOND|MULTI. `listener` is finite, so we re-issue it whenever
    the IR stream goes stale.
    """

    DEV_SHELL = mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL
    FLAGS = (mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND
             | mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
             | mavutil.mavlink.SERIAL_CONTROL_FLAG_MULTI)

    def __init__(self, master, topic, batch):
        self.master = master
        self.cmd = f"listener {topic} {batch}\n"
        self.buf = ""
        self.rec = {}
        self.last_issue = 0.0

    def _send(self, text):
        data = text.encode()
        while data:
            chunk, data = data[:70], data[70:]
            payload = list(chunk) + [0] * (70 - len(chunk))
            self.master.mav.serial_control_send(
                self.DEV_SHELL, self.FLAGS, 0, 0, len(chunk), payload)

    def poll(self):
        """Keep the pipe open and re-issue the listener command when stale."""
        now = time.monotonic()
        with STATE.lock:
            ir_age = now - STATE.ir_t if STATE.ir_t else 1e9
        if ir_age > 1.5 and now - self.last_issue > 1.5:
            self.last_issue = now
            self._send("\n")        # nudge the prompt
            self._send(self.cmd)
        else:
            # empty RESPOND keeps PX4 pushing buffered output
            self.master.mav.serial_control_send(
                self.DEV_SHELL, self.FLAGS, 0, 0, 0, [0] * 70)

    # -- parsing ------------------------------------------------------------
    FIELD = re.compile(r"^\s*([a-z_]+):\s*(-?[\w.+-]+)")

    @staticmethod
    def _num(tok):
        t = tok.lower()
        if t in ("nan", "-nan", "+nan"):
            return float("nan")
        if t in ("true", "false"):
            return t == "true"
        try:
            return float(tok)
        except ValueError:
            return None

    def feed(self, blob):
        self.buf += blob
        *lines, self.buf = self.buf.split("\n")
        for raw in lines:
            STATE.shell_lines += 1
            m = self.FIELD.match(raw)
            if not m:
                continue
            key, tok = m.group(1), m.group(2)
            val = self._num(tok)
            if val is None:
                continue
            self.rec[key] = val
            # `valid` is the last field of IrCameraReport -> record complete
            if key == "valid" and "angle_x" in self.rec:
                self._emit()
                self.rec = {}

    def _emit(self):
        r = self.rec
        with STATE.lock:
            STATE.dx_px = r.get("dx_px", 0.0)
            STATE.dy_px = r.get("dy_px", 0.0)
            STATE.angle_x = r.get("angle_x", float("nan"))
            STATE.angle_y = r.get("angle_y", float("nan"))
            STATE.valid = bool(r.get("valid", False))
            STATE.spot_id = int(r.get("spot_id", 0))
            STATE.spot_score = int(r.get("spot_score", 0))
            STATE.frame_w = int(r.get("frame_width", 1236))
            STATE.frame_h = int(r.get("frame_height", 960))
            STATE.ir_t = time.monotonic()
            STATE.ir_count += 1
            STATE.ir_via = "shell"


IRCAM_ARRAY_NAME = "IRCAM"       # matches IrCam::publishDebugArray()


def feed_debug_array(msg):
    """Unpack DEBUG_FLOAT_ARRAY 'IRCAM' -> STATE.

    Layout is fixed by IrCam::publishDebugArray():
      0 dx_px  1 dy_px  2 angle_x  3 angle_y  4 valid  5 spot_id  6 spot_score
    Returns True if this was our array.
    """
    name = bytes(msg.name).split(b"\x00")[0].decode("ascii", "replace") \
        if isinstance(msg.name, (bytes, bytearray, list)) else str(msg.name).rstrip("\x00")
    if name != IRCAM_ARRAY_NAME:
        return False
    d = msg.data
    with STATE.lock:
        STATE.dx_px = d[0]
        STATE.dy_px = d[1]
        STATE.angle_x = d[2]
        STATE.angle_y = d[3]
        STATE.valid = d[4] > 0.5
        STATE.spot_id = int(d[5])
        STATE.spot_score = int(d[6])
        STATE.ir_t = time.monotonic()
        STATE.ir_count += 1
        STATE.ir_via = "DEBUG_FLOAT_ARRAY"
    return True


def request_streams(master):
    """Ask for the telemetry we need at a useful rate."""
    wanted = [(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20),
              (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20),
              (mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 10),
              (mavutil.mavlink.MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY, 30)]
    for msg_id, hz in wanted:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)


def mav_thread(args):
    port = args.connect
    if port in (None, "auto"):
        port = autodetect_port()
        if not port:
            STATE.last_err = "no USB device found; pass --connect"
            print("!! " + STATE.last_err)
            STOP.set()
            return
    is_serial = not port.startswith(("udp:", "udpin:", "udpout:", "tcp:"))
    print(f"connecting to {port}" + (f" @ {args.baud}" if is_serial else "") + " ...")
    try:
        master = (mavutil.mavlink_connection(port, baud=args.baud) if is_serial
                  else mavutil.mavlink_connection(port))
        master.wait_heartbeat(timeout=15)
    except Exception as exc:                                  # noqa: BLE001
        STATE.last_err = f"connect failed: {exc}"
        print("!! " + STATE.last_err)
        STOP.set()
        return
    print(f"heartbeat from sys {master.target_system} comp {master.target_component}")
    with STATE.lock:
        STATE.connected = True
        STATE.tgt_sys = master.target_system
        STATE.tgt_comp = master.target_component or 1

    request_streams(master)

    # Prefer DEBUG_FLOAT_ARRAY from the IR_CAM_DEBUG_MAVLINK firmware patch.
    # Only drop back to scraping the nsh console if it never shows up, since
    # that path is slow and fights QGC for the shell.
    shell = None
    if args.no_ir:
        print("IR disabled (--no-ir): motion only")
    elif args.ir_source == "shell":
        shell = ShellReader(master, args.topic, args.batch)
        print("IR via nsh console (forced)")
    elif args.ir_source == "array":
        print(f"IR via DEBUG_FLOAT_ARRAY '{IRCAM_ARRAY_NAME}' (forced)")
    else:
        print(f"IR: waiting for DEBUG_FLOAT_ARRAY '{IRCAM_ARRAY_NAME}', "
              f"falling back to nsh console after {args.array_wait:.0f}s")
    t_start = time.monotonic()
    fell_back = False

    last_hb = last_poll = last_req = 0.0
    while not STOP.is_set():
        now = time.monotonic()

        if (not args.no_ir and args.ir_source == "auto" and shell is None
                and not fell_back and now - t_start > args.array_wait):
            with STATE.lock:
                seen = STATE.ir_count
            if seen == 0:
                fell_back = True
                shell = ShellReader(master, args.topic, args.batch)
                print("!! no DEBUG_FLOAT_ARRAY - is IR_CAM_DEBUG_MAVLINK flashed?"
                      "  falling back to nsh console (close QGC)")
            else:
                fell_back = True   # array works, never start the shell
                print("IR via DEBUG_FLOAT_ARRAY - console not needed")

        if now - last_hb > 1.0:
            last_hb = now
            master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                      mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        if now - last_req > 5.0:
            last_req = now
            request_streams(master)

        # drain arm/disarm requests posted by the UI thread
        while True:
            try:
                want_arm, force = CMD_Q.get_nowait()
            except queue.Empty:
                break
            with STATE.lock:
                ts, tc = STATE.tgt_sys, STATE.tgt_comp
                STATE.ack = f"sending {'ARM' if want_arm else 'DISARM'} ..."
                STATE.ack_t = now
            try:
                master.mav.command_long_send(
                    ts, tc, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                    1.0 if want_arm else 0.0, 21196.0 if force else 0.0,
                    0, 0, 0, 0, 0)
            except Exception as exc:                          # noqa: BLE001
                with STATE.lock:
                    STATE.ack = f"send failed: {exc}"
                    STATE.ack_t = now
        if shell and now - last_poll > 0.2:
            last_poll = now
            try:
                shell.poll()
            except Exception as exc:                          # noqa: BLE001
                STATE.last_err = f"shell: {exc}"

        msg = master.recv_match(blocking=True, timeout=0.1)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "DEBUG_FLOAT_ARRAY":
            feed_debug_array(msg)
        elif t == "SERIAL_CONTROL" and shell and msg.count:
            shell.feed(bytes(msg.data[:msg.count]).decode("utf8", "replace"))
        elif t == "ATTITUDE":
            with STATE.lock:
                STATE.roll, STATE.pitch, STATE.yaw = msg.roll, msg.pitch, msg.yaw
                STATE.rollspeed = msg.rollspeed
                STATE.pitchspeed = msg.pitchspeed
                STATE.yawspeed = msg.yawspeed
                STATE.att_t = now
        elif t == "LOCAL_POSITION_NED":
            with STATE.lock:
                STATE.x, STATE.y = msg.x, msg.y
                STATE.vn, STATE.ve = msg.vx, msg.vy
                STATE.lp_t = now
                if STATE.origin is None:
                    STATE.origin = (msg.x, msg.y)
        elif t == "DISTANCE_SENSOR":
            with STATE.lock:
                STATE.height = msg.current_distance / 100.0
        elif t == "HEARTBEAT":
            # only the autopilot's own heartbeat carries the arm state
            if msg.get_srcSystem() == master.target_system and msg.type != \
                    mavutil.mavlink.MAV_TYPE_GCS:
                with STATE.lock:
                    STATE.armed = bool(msg.base_mode
                                       & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    STATE.hb_t = now
        elif t == "COMMAND_ACK":
            if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                with STATE.lock:
                    STATE.ack = ACK_RESULT.get(msg.result, f"result {msg.result}")
                    STATE.ack_t = now


def demo_thread(args):
    """Synthetic data so the UI can be checked with no drone attached.

    Simulates a beacon 0.35 m ahead / 0.20 m right of the start point while the
    vehicle creeps forward-right at ~0.12 m/s and slowly yaws.
    """
    print("DEMO MODE - synthetic data, nothing is connected")
    with STATE.lock:
        STATE.connected = True
    t0 = time.monotonic()
    bn, be = 0.35, 0.20          # beacon in NED relative to start
    x = y = 0.0
    while not STOP.is_set():
        time.sleep(1 / 15)
        now = time.monotonic()
        t = now - t0
        # fake autopilot heartbeat + arm handling so the button is exercisable
        with STATE.lock:
            STATE.hb_t = now
        while True:
            try:
                want_arm, _force = CMD_Q.get_nowait()
            except queue.Empty:
                break
            with STATE.lock:
                STATE.armed = want_arm
                STATE.ack = "ACCEPTED (demo)"
                STATE.ack_t = now
        vn, ve = 0.12, 0.06
        x, y = vn * t, ve * t
        yaw = math.radians(20 * math.sin(t / 6))
        h = 0.7
        # beacon offset in body frame -> angles
        on, oe = bn - x, be - y
        of = on * math.cos(yaw) + oe * math.sin(yaw)
        orr = -on * math.sin(yaw) + oe * math.cos(yaw)
        with STATE.lock:
            STATE.angle_x = math.atan2(of, h)
            STATE.angle_y = math.atan2(orr, h)
            STATE.dx_px = -math.tan(STATE.angle_y) * args.fx
            STATE.dy_px = math.tan(STATE.angle_x) * args.fy
            STATE.valid = abs(of) < h * 0.45 and abs(orr) < h * 0.59
            STATE.spot_id, STATE.spot_score = 0, 127
            STATE.ir_t = now
            STATE.ir_count += 1
            STATE.yaw, STATE.vn, STATE.ve = yaw, vn, ve
            STATE.x, STATE.y, STATE.height = x, y, h
            STATE.att_t = STATE.lp_t = now
            if STATE.origin is None:
                STATE.origin = (x, y)


# ----------------------------------------------------------------------------
# view
# ----------------------------------------------------------------------------

class Monitor:
    def __init__(self, args):
        self.args = args
        self.half_ax = math.atan(args.cy / args.fy)   # fore/aft half-FOV [rad]
        self.half_ay = math.atan(args.cx / args.fx)   # lateral half-FOV [rad]
        self.gain = 1.0
        self.trail = deque(maxlen=args.trail)
        self.t0 = time.monotonic()
        self.last_frame = self.t0
        self.csv = None
        self.csv_fh = None
        self.csv_rows = 0
        if args.csv:
            self.csv_fh = open(args.csv, "w", newline="")
            self.csv = csv.writer(self.csv_fh)
            # roll/pitch are essential: without them the beacon offset cannot be
            # tilt-compensated after the fact, and at ~1 m height a 10 deg tilt
            # is ~0.17 m of phantom offset -- as big as the signal.
            self.csv.writerow(["t", "dx_px", "dy_px", "angle_x", "angle_y", "valid",
                               "spot_id", "spot_score", "height",
                               "roll_deg", "pitch_deg", "yaw_deg",
                               "rollspeed", "pitchspeed", "yawspeed",
                               "vn", "ve", "v_fwd", "v_right", "x", "y",
                               "disp_n", "disp_e"])

        R = args.range
        self.fig, self.ax = plt.subplots(figsize=(8.6, 9.2))
        try:
            self.fig.canvas.manager.set_window_title("IR target + optical flow (body frame)")
        except Exception:                                     # noqa: BLE001
            pass                                              # headless backend
        self.ax.set_xlim(-R, R)
        self.ax.set_ylim(-R, R)
        self.ax.set_aspect("equal")
        self.ax.set_facecolor("#0e1116")
        self.ax.set_xlabel("body +Y  ->  RIGHT   [m]", color="#c8d0da")
        self.ax.set_ylabel("body +X  ->  NOSE / FORWARD   [m]", color="#c8d0da")
        self.ax.tick_params(colors="#7b8794")
        for s in self.ax.spines.values():
            s.set_color("#2a313c")
        self.ax.grid(color="#1d232c", lw=0.7)
        self.ax.axhline(0, color="#2f3947", lw=1)
        self.ax.axvline(0, color="#2f3947", lw=1)

        # drifting ground dots (what the optical flow sees)
        rng = np.random.default_rng(1)
        self.dots = rng.uniform(-R, R, size=(args.dots, 2))
        self.dot_art = self.ax.scatter(self.dots[:, 0], self.dots[:, 1],
                                       s=7, c="#3f7fa8", alpha=0.75, zorder=2)

        self.fov = Rectangle((0, 0), 0, 0, fill=False, ls="--", lw=1.4,
                             ec="#c8a13a", zorder=3)
        self.ax.add_patch(self.fov)

        self.trail_art, = self.ax.plot([], [], "-", color="#8a3f5a", lw=1.4,
                                       alpha=0.9, zorder=4)
        self.tgt_art, = self.ax.plot([], [], "o", ms=17, mec="white", mew=1.6,
                                     color="#e2364a", zorder=6)
        self.vel_art = self.ax.annotate(
            "", xy=(0, 0), xytext=(0, 0), zorder=7,
            arrowprops=dict(arrowstyle="-|>", lw=3.0, color="#37d67a",
                            shrinkA=0, shrinkB=0))
        # drone marker at centre
        self.ax.plot([0], [0], marker="+", ms=18, mew=2.2, color="#dfe6ee", zorder=5)
        self.ax.annotate("NOSE", xy=(0, R * 0.94), ha="center", color="#5f6b7a",
                         fontsize=9, zorder=5)

        self.hud = self.ax.text(
            -R * 0.985, R * 0.985, "", va="top", ha="left", family="monospace",
            fontsize=9.0, color="#d4dbe4", zorder=10,
            bbox=dict(fc="#141a21", ec="#2a313c", alpha=0.94, pad=6))
        self.warn = self.ax.text(
            0, -R * 0.93, "", ha="center", family="monospace", fontsize=11.5,
            color="#ffb020", zorder=10)

        # ---- arm / disarm button --------------------------------------
        # Label and colour always follow the REAL state from HEARTBEAT, never
        # what we last asked for -- a rejected arm must not look armed.
        self.arm_pending_until = 0.0     # arm needs a second click to confirm
        self.btn_ax = self.fig.add_axes([0.035, 0.022, 0.26, 0.055])
        self.btn = Button(self.btn_ax, "---", color="#2a313c", hovercolor="#3a444f")
        self.btn.label.set_fontsize(12)
        self.btn.label.set_fontweight("bold")
        self.btn.on_clicked(self.on_arm_click)
        self.ack_txt = self.fig.text(0.315, 0.048, "", va="center", ha="left",
                                     family="monospace", fontsize=9.5,
                                     color="#9aa5b1")

        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.fig.subplots_adjust(left=0.10, right=0.98, top=0.98, bottom=0.115)

    # ------------------------------------------------------------------
    def on_arm_click(self, _ev):
        """Disarm goes through immediately; arm needs a confirming second click.

        The asymmetry is deliberate: disarm is always the safe direction, so it
        must never be gated. Arm spins props, so a stray click must not do it.
        """
        now = time.monotonic()
        with STATE.lock:
            armed, live = STATE.armed, (now - STATE.hb_t) < 3.0
        if not live:
            with STATE.lock:
                STATE.ack, STATE.ack_t = "no autopilot heartbeat", now
            return
        if armed:
            self.arm_pending_until = 0.0
            CMD_Q.put((False, self.args.force_disarm))
            return
        if now < self.arm_pending_until:          # confirmed
            self.arm_pending_until = 0.0
            CMD_Q.put((True, False))
        else:
            self.arm_pending_until = now + 3.0    # ask for confirmation

    def refresh_arm_button(self, now):
        with STATE.lock:
            armed, live = STATE.armed, (now - STATE.hb_t) < 3.0
            ack, ack_age = STATE.ack, now - STATE.ack_t
        if not live:
            label, col = "NO LINK", "#2a313c"
            self.arm_pending_until = 0.0
        elif armed:
            label, col = "DISARM", "#b3202f"
        elif now < self.arm_pending_until:
            label, col = f"CONFIRM ARM {self.arm_pending_until - now:.0f}", "#c2760f"
        else:
            label, col = "ARM", "#1d6b3a"
        if self.btn.label.get_text() != label:
            self.btn.label.set_text(label)
        if self.btn_ax.get_facecolor() != col:
            self.btn.color = col
            self.btn_ax.set_facecolor(col)
        self.ack_txt.set_text(
            f"{'ARMED' if armed else 'disarmed'}"
            + (f"   |   {ack}" if ack and ack_age < 6.0 else ""))
        self.ack_txt.set_color("#ff6b6b" if armed else "#9aa5b1")

    def on_key(self, ev):
        if ev.key == "q":
            STOP.set()
            plt.close(self.fig)
        elif ev.key == "r":
            with STATE.lock:
                STATE.origin = (STATE.x, STATE.y)
            self.trail.clear()
        elif ev.key == "c":
            self.trail.clear()
        elif ev.key in ("+", "="):
            self.gain = min(self.gain * 2, 64)
        elif ev.key == "-":
            self.gain = max(self.gain / 2, 1 / 8)

    # ------------------------------------------------------------------
    def update(self, _):
        now = time.monotonic()
        dt = min(now - self.last_frame, 0.25)
        self.last_frame = now
        R = self.args.range
        self.refresh_arm_button(now)

        with STATE.lock:
            s = STATE
            ir_age = now - s.ir_t if s.ir_t else 1e9
            ax_, ay_ = s.angle_x, s.angle_y
            valid, dx, dy = s.valid, s.dx_px, s.dy_px
            sid, score = s.spot_id, s.spot_score
            yaw, vn, ve = s.yaw, s.vn, s.ve
            roll, pitch = s.roll, s.pitch
            rollspeed, pitchspeed, yawspeed = s.rollspeed, s.pitchspeed, s.yawspeed
            hgt = s.height
            x, y = s.x, s.y
            origin = s.origin
            n_ir = s.ir_count
            ir_via = s.ir_via
            err = s.last_err
            lp_age = now - s.lp_t if s.lp_t else 1e9

        h = hgt if (hgt == hgt and hgt > 0.05) else self.args.height

        # body-frame velocity from NED + yaw
        v_fwd = vn * math.cos(yaw) + ve * math.sin(yaw)
        v_right = -vn * math.sin(yaw) + ve * math.cos(yaw)

        # ---- ground dots drift OPPOSITE the vehicle motion (that is the flow)
        self.dots[:, 0] -= v_right * dt * self.gain
        self.dots[:, 1] -= v_fwd * dt * self.gain
        self.dots = ((self.dots + R) % (2 * R)) - R
        self.dot_art.set_offsets(self.dots)

        # ---- camera footprint at this height
        fx_m, fy_m = h * math.tan(self.half_ay), h * math.tan(self.half_ax)
        self.fov.set_bounds(-fx_m, -fy_m, 2 * fx_m, 2 * fy_m)

        # ---- target
        fresh = ir_age < 1.0
        off_f = off_r = float("nan")
        if fresh and valid and ax_ == ax_ and ay_ == ay_:
            off_f = h * math.tan(ax_)      # forward, screen +Y
            off_r = h * math.tan(ay_)      # right,   screen +X
            self.tgt_art.set_data([off_r], [off_f])
            self.tgt_art.set_color("#e2364a")
            self.trail.append((off_r, off_f))
        else:
            self.tgt_art.set_data([], [])
        if self.trail:
            t_arr = np.asarray(self.trail)
            self.trail_art.set_data(t_arr[:, 0], t_arr[:, 1])

        # ---- velocity arrow (hidden below ~1 mm/s so matplotlib gets no
        #      zero-length arrow, which it renders as an artifact)
        k = self.args.vel_scale
        if math.hypot(v_fwd, v_right) > 1e-3:
            self.vel_art.set_visible(True)
            self.vel_art.set_position((0, 0))
            self.vel_art.xy = (v_right * k, v_fwd * k)
        else:
            self.vel_art.set_visible(False)

        # ---- displacement since origin
        dn = de = float("nan")
        if origin:
            dn, de = x - origin[0], y - origin[1]
        d_fwd = dn * math.cos(yaw) + de * math.sin(yaw) if dn == dn else float("nan")
        d_right = -dn * math.sin(yaw) + de * math.cos(yaw) if dn == dn else float("nan")

        lsb = 0.213 * h        # flow velocity quantisation at this height
        self.hud.set_text(
            f"IR   {'VALID' if (fresh and valid) else 'NO TARGET':9s} age {min(ir_age,99):5.2f}s  "
            f"n={n_ir} via {ir_via}\n"
            f"  dx/dy px   {dx:+8.1f} {dy:+8.1f}   id={sid} score={score}\n"
            f"  angle x/y  {ax_:+8.3f} {ay_:+8.3f} rad\n"
            f"  offset f/r {off_f:+8.3f} {off_r:+8.3f} m\n"
            f"  height     {h:8.2f} m   FOV +/-{fy_m:.2f} fwd  +/-{fx_m:.2f} lat\n"
            f"MOTION  age {min(lp_age,99):5.2f}s   yaw {math.degrees(yaw):+7.1f} deg\n"
            f"  vel NED    {vn:+8.3f} {ve:+8.3f} m/s\n"
            f"  vel body   {v_fwd:+8.3f} {v_right:+8.3f} m/s  (fwd,right)\n"
            f"  disp body  {d_fwd:+8.3f} {d_right:+8.3f} m   [r]=reset\n"
            f"  flow LSB   {lsb:8.3f} m/s  <- resolution floor\n"
            f"dots x{self.gain:g}  +/- to change"
        )

        msgs = []
        if err:
            msgs.append(err)
        if fresh and valid and off_f == off_f and (abs(off_f) > fy_m or abs(off_r) > fx_m):
            msgs.append("target at FOV edge - about to be lost")
        if not fresh:
            msgs.append("no IR data - is QGC holding the console?")
        if abs(v_fwd) < lsb * 0.5 and abs(v_right) < lsb * 0.5:
            msgs.append(f"|v| below flow resolution ({lsb:.2f} m/s) - flow is blind here")
        self.warn.set_text("   ".join(msgs[:2]))

        if self.csv and fresh:
            self.csv.writerow([f"{now - self.t0:.3f}", dx, dy, ax_, ay_, int(valid),
                               sid, score, f"{h:.3f}",
                               f"{math.degrees(roll):.2f}", f"{math.degrees(pitch):.2f}",
                               f"{math.degrees(yaw):.2f}",
                               f"{rollspeed:.4f}", f"{pitchspeed:.4f}", f"{yawspeed:.4f}",
                               f"{vn:.4f}", f"{ve:.4f}", f"{v_fwd:.4f}", f"{v_right:.4f}",
                               f"{x:.4f}", f"{y:.4f}",
                               f"{dn:.4f}" if dn == dn else "", f"{de:.4f}" if de == de else ""])
            # flush often: a Ctrl-C or window close must not lose the run
            self.csv_rows += 1
            if self.csv_rows % 10 == 0:
                self.csv_fh.flush()

        return ()

    def close(self):
        if self.csv_fh:
            self.csv_fh.flush()
            self.csv_fh.close()
            print(f"wrote {self.csv_rows} rows to {self.args.csv}")


SAMPLE = """nsh> listener ir_camera_report

TOPIC: ir_camera_report
 ir_camera_report
    timestamp: 37941852 (0.007137 seconds ago)
    dx_px: -68.00000
    dy_px: 287.00000
    angle_x: 0.26907
    angle_y: 0.06636
    spot_score: 16
    frame_width: 1236
    frame_height: 960
    spot_id: 2
    valid: True

TOPIC: ir_camera_report instance 0 #5
 ir_camera_report
    timestamp: 58326819 (0.085656 seconds ago)
    dx_px: 79.00000
    dy_px: -433.00000
    angle_x: nan
    angle_y: nan
    spot_score: 127
    frame_width: 1236
    frame_height: 960
    spot_id: 1
    valid: False
"""


def selftest():
    """Feed real `listener` output through the parser, byte-chunked like MAVLink."""
    got = []
    sh = ShellReader.__new__(ShellReader)      # no MAVLink connection needed
    sh.buf, sh.rec = "", {}
    orig = ShellReader._emit

    def capture(self):
        got.append(dict(self.rec))
        orig(self)
    ShellReader._emit = capture
    try:
        for i in range(0, len(SAMPLE), 70):    # 70 B = SERIAL_CONTROL payload
            sh.feed(SAMPLE[i:i + 70])
    finally:
        ShellReader._emit = orig

    ok = True
    if len(got) != 2:
        print(f"FAIL expected 2 records, got {len(got)}")
        return 1
    a, b = got
    checks = [
        ("rec0 dx_px", a["dx_px"], -68.0), ("rec0 dy_px", a["dy_px"], 287.0),
        ("rec0 angle_x", a["angle_x"], 0.26907), ("rec0 angle_y", a["angle_y"], 0.06636),
        ("rec0 valid", a["valid"], True), ("rec0 spot_id", a["spot_id"], 2.0),
        ("rec0 spot_score", a["spot_score"], 16.0),
        ("rec1 dy_px", b["dy_px"], -433.0), ("rec1 valid", b["valid"], False),
    ]
    for name, actual, want in checks:
        good = (actual == want)
        ok &= good
        print(f"  {'ok  ' if good else 'FAIL'} {name}: {actual!r} (want {want!r})")
    nan_ok = b["angle_x"] != b["angle_x"] and b["angle_y"] != b["angle_y"]
    ok &= nan_ok
    print(f"  {'ok  ' if nan_ok else 'FAIL'} rec1 angles parsed as NaN")

    # --- DEBUG_FLOAT_ARRAY path: build a real message and round-trip it ------
    print("\nDEBUG_FLOAT_ARRAY unpack:")
    data = [0.0] * 58
    data[0:7] = [-68.0, 287.0, 0.26907, 0.06636, 1.0, 2.0, 16.0]
    mav = mavutil.mavlink.MAVLink(None)
    m = mavutil.mavlink.MAVLink_debug_float_array_message(
        time_usec=123456, name=b"IRCAM", array_id=0, data=data)
    m.pack(mav)                                   # exercise real encoding
    STATE.ir_count = 0
    hit = feed_debug_array(m)
    ok &= hit
    print(f"  {'ok  ' if hit else 'FAIL'} recognised name 'IRCAM'")
    for name, actual, want in [("dx_px", STATE.dx_px, -68.0),
                               ("dy_px", STATE.dy_px, 287.0),
                               ("angle_x", round(STATE.angle_x, 5), 0.26907),
                               ("angle_y", round(STATE.angle_y, 5), 0.06636),
                               ("valid", STATE.valid, True),
                               ("spot_id", STATE.spot_id, 2),
                               ("spot_score", STATE.spot_score, 16),
                               ("via", STATE.ir_via, "DEBUG_FLOAT_ARRAY")]:
        good = actual == want
        ok &= good
        print(f"  {'ok  ' if good else 'FAIL'} {name}: {actual!r} (want {want!r})")
    other = mavutil.mavlink.MAVLink_debug_float_array_message(
        time_usec=1, name=b"OTHER", array_id=0, data=data)
    ignored = not feed_debug_array(other)
    ok &= ignored
    print(f"  {'ok  ' if ignored else 'FAIL'} ignores foreign array name 'OTHER'")

    print("\nPASS" if ok else "\nFAIL")
    return 0 if ok else 1


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--connect", default="auto",
                   help="serial device, or udp:127.0.0.1:14550 (default: auto-detect USB)")
    p.add_argument("--baud", type=int, default=57600, help="serial baud (USB CDC ignores it)")
    p.add_argument("--topic", default="ir_camera_report", help="uORB topic to listen to")
    p.add_argument("--batch", type=int, default=200,
                   help="samples per `listener` invocation before re-issuing")
    p.add_argument("--no-ir", action="store_true", help="motion only, do not touch the shell")
    p.add_argument("--ir-source", choices=("auto", "array", "shell"), default="auto",
                   help="auto: prefer DEBUG_FLOAT_ARRAY, fall back to nsh console; "
                        "array: DEBUG_FLOAT_ARRAY only (needs IR_CAM_DEBUG_MAVLINK "
                        "firmware); shell: console scraping only")
    p.add_argument("--array-wait", type=float, default=3.0,
                   help="seconds to wait for DEBUG_FLOAT_ARRAY before falling back")
    p.add_argument("--range", type=float, default=1.5, help="half-width of the view [m]")
    p.add_argument("--height", type=float, default=0.7,
                   help="fallback height if DISTANCE_SENSOR is absent [m]")
    p.add_argument("--dots", type=int, default=220, help="number of ground dots")
    p.add_argument("--vel-scale", type=float, default=2.0,
                   help="metres of arrow per m/s")
    p.add_argument("--trail", type=int, default=150, help="beacon trail length")
    p.add_argument("--fx", type=float, default=1047.0)
    p.add_argument("--fy", type=float, default=1065.0)
    p.add_argument("--cx", type=float, default=618.0)
    p.add_argument("--cy", type=float, default=480.0)
    p.add_argument("--csv", help="also record everything to this CSV")
    p.add_argument("--force-disarm", action="store_true",
                   help="use the force flag (21196) when disarming, so it works "
                        "in flight too. Off by default.")
    p.add_argument("--demo", action="store_true",
                   help="synthetic data, no drone needed (checks the UI)")
    p.add_argument("--selftest", action="store_true",
                   help="parse a canned listener sample and exit")
    args = p.parse_args()

    if args.selftest:
        return selftest()

    target = demo_thread if args.demo else mav_thread
    th = threading.Thread(target=target, args=(args,), daemon=True)
    th.start()

    mon = Monitor(args)
    anim = matplotlib.animation.FuncAnimation(
        mon.fig, mon.update, interval=50, blit=False, cache_frame_data=False)
    mon._anim = anim  # keep a reference alive
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        STOP.set()
        mon.close()
    th.join(timeout=2)
    print("done")


if __name__ == "__main__":
    main()
