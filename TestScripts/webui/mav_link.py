#!/usr/bin/env python3
"""MAVLink link + telemetry state for the rope-hung drone web UI.

One receive thread owns the pymavlink connection. Everyone else either

  * reads a JSON-safe copy of the state with :meth:`MavLink.snapshot`, or
  * posts a request (arm / disarm / param) that the receive thread drains.

pymavlink connections are NOT safe for concurrent sends, hence the queue and
the public :attr:`MavLink.send_lock` (``log_download.py`` holds it while it
sends its own LOG_REQUEST_* messages from the receive thread).

Contract: see ``TestScripts/WEBUI_CONTRACT.md``. The STATE schema there is the
single source of truth; this module implements it. Two documented additions,
both additive so nothing in the schema changes shape:

  * ``link.error`` -- last link-level error string ("" when healthy). A field
    tool has to be able to say *why* it is not connected.
  * ``ack`` is ``null`` until the first COMMAND_ACK (or arm/disarm attempt)
    rather than a fake all-zero ack, so the UI can tell "nothing yet" from
    "the vehicle said ACCEPTED".

Times (``t``, ``messages[].t``, ``ack.t``) are ``time.monotonic()`` seconds,
i.e. only meaningful relative to the ``t`` of the same snapshot. Ages are
already computed for you; prefer them.

Selftest (no hardware needed):

    python3 mav_link.py --selftest

Live smoke test:

    python3 mav_link.py --connect udpin:0.0.0.0:14550 --dump
"""

import argparse
import glob
import json
import math
import os
import queue
import struct
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
except ImportError:                                            # pragma: no cover
    sys.exit("pymavlink missing:  pip install pymavlink")

mavlink = mavutil.mavlink

if not hasattr(mavlink, "MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY"):   # pragma: no cover
    sys.exit("pymavlink loaded a MAVLink 1 dialect without DEBUG_FLOAT_ARRAY.\n"
             "Run with:  MAVLINK20=1 MAVLINK_DIALECT=common python3 ...")


# ----------------------------------------------------------------------------
# constants
# ----------------------------------------------------------------------------

#: (message id, Hz) requested via MAV_CMD_SET_MESSAGE_INTERVAL, re-sent every 5 s
STREAMS = (
    (mavlink.MAVLINK_MSG_ID_ATTITUDE, 20),
    (mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20),
    (mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 10),
    (mavlink.MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY, 30),
    (mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2),
    (mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 2),
    (mavlink.MAVLINK_MSG_ID_VFR_HUD, 5),
    (mavlink.MAVLINK_MSG_ID_ESTIMATOR_STATUS, 2),
)

ACK_RESULT = {
    0: "ACCEPTED", 1: "TEMPORARILY REJECTED", 2: "DENIED", 3: "UNSUPPORTED",
    4: "FAILED", 5: "IN PROGRESS", 6: "CANCELLED",
}

#: matches IrCam::publishDebugArray() -- index -> field
IRCAM_ARRAY_NAME = "IRCAM"

STREAM_INTERVAL = 5.0      # re-request streams every N s
HEARTBEAT_INTERVAL = 1.0   # our GCS heartbeat
LINK_TIMEOUT = 3.0         # vehicle heartbeat age beyond which we are "not connected"
SILENCE_REOPEN = 6.0       # no traffic at all for this long -> reopen the link
PARAM_LIST_MIN_GAP = 3.0   # don't spam PARAM_REQUEST_LIST
RECONNECT_BACKOFF = (0.5, 1.0, 2.0, 3.0, 5.0)
MAX_MESSAGES = 100
AGE_NEVER = 999.9          # reported age for "never received"

# PX4 encodes int parameters bytewise: the int32 bits are memcpy'd into the
# float param_value field (see MavlinkParametersManager::send_param, and the
# matching param_set(param, &set.param_value) on the way in). Reading the field
# as a float would show DF_SWAY_EN=1 as 1.4e-45. It also rejects a PARAM_SET
# whose param_type does not match the parameter's real type, so we remember the
# type from PARAM_VALUE and echo it back.
_INT_PARAM_FMT = {
    mavlink.MAV_PARAM_TYPE_UINT8: "<B",
    mavlink.MAV_PARAM_TYPE_INT8: "<b",
    mavlink.MAV_PARAM_TYPE_UINT16: "<H",
    mavlink.MAV_PARAM_TYPE_INT16: "<h",
    mavlink.MAV_PARAM_TYPE_UINT32: "<I",
    mavlink.MAV_PARAM_TYPE_INT32: "<i",
}


# ----------------------------------------------------------------------------
# helpers
# ----------------------------------------------------------------------------

def json_safe(obj):
    """Recursively replace every non-finite float with ``None``.

    Bare ``NaN`` / ``Infinity`` are not valid JSON: ``json.dumps`` emits them
    happily but ``JSON.parse`` in the browser throws, killing the whole SSE
    stream. Everything leaving this module goes through here.
    """
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, bool) or isinstance(obj, int) or obj is None:
        return obj
    if isinstance(obj, dict):
        return {k: json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple, set, frozenset, deque)):
        return [json_safe(v) for v in obj]
    return obj


def autodetect_port():
    """Pick the most likely PX4 USB CDC-ACM device for this OS."""
    pats = ["/dev/cu.usbmodem*", "/dev/tty.usbmodem*",   # macOS
            "/dev/serial/by-id/*PX4*", "/dev/ttyACM*"]    # Linux
    for p in pats:
        hits = sorted(glob.glob(p))
        if hits:
            return hits[0]
    return None


def is_network_address(address):
    return str(address).startswith(("udp:", "udpin:", "udpout:", "udpbcast:",
                                    "tcp:", "tcpin:"))


def is_listening_address(address):
    """True for links that bind and wait (mavutil treats bare ``udp:`` as input).

    A listener must not be torn down just because the far end went quiet: that
    is the normal MAVProxy-restart case and the socket is still perfectly good.
    """
    return str(address).startswith(("udpin:", "udp:", "tcpin:"))


def array_name(msg):
    """DEBUG_FLOAT_ARRAY / any char[] name -> clean str, whatever pymavlink gave us."""
    n = getattr(msg, "name", "")
    if isinstance(n, (bytes, bytearray)):
        raw = bytes(n)
    elif isinstance(n, (list, tuple)):
        try:
            raw = bytes(int(c) & 0xFF for c in n)
        except (TypeError, ValueError):
            raw = b"".join(bytes(str(c), "ascii", "replace") for c in n)
    else:
        return str(n).split("\x00")[0].strip()
    return raw.split(b"\x00")[0].decode("ascii", "replace").strip()


def as_text(value):
    """STATUSTEXT text -> str, tolerating bytes and embedded NULs."""
    if isinstance(value, (bytes, bytearray)):
        value = bytes(value).decode("utf-8", "replace")
    return str(value).split("\x00")[0].strip()


def trim_array(data, keep=8):
    """Floats with trailing zeros removed, but never shorter than ``keep``."""
    vals = []
    for v in data:
        try:
            vals.append(float(v))
        except (TypeError, ValueError):
            vals.append(float("nan"))
    n = len(vals)
    while n > keep and vals[n - 1] == 0.0:
        n -= 1
    return vals[:n]


def safe_int(value, default=0):
    try:
        if not math.isfinite(float(value)):
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def decode_param_value(value, param_type):
    """PARAM_VALUE.param_value -> real number, undoing PX4's bytewise int trick."""
    fmt = _INT_PARAM_FMT.get(param_type)
    if fmt is None:
        try:
            return float(value)
        except (TypeError, ValueError):
            return float("nan")
    try:
        raw = struct.pack("<f", float(value))
        return float(struct.unpack(fmt, raw[:struct.calcsize(fmt)])[0])
    except (struct.error, TypeError, ValueError, OverflowError):
        try:
            return float(value)
        except (TypeError, ValueError):
            return float("nan")


def encode_param_value(value, param_type):
    """Real number -> the float field PX4 expects for this parameter type."""
    fmt = _INT_PARAM_FMT.get(param_type)
    if fmt is None:
        return float(value)
    try:
        raw = struct.pack(fmt, int(round(float(value))))
        raw = raw + b"\x00" * (4 - len(raw))
        return struct.unpack("<f", raw)[0]
    except (struct.error, TypeError, ValueError, OverflowError):
        return float(value)


#: PX4 custom_mode: main mode in byte 2, sub mode in byte 3
PX4_MAIN_MODES = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO", 5: "ACRO",
                  6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE"}
PX4_AUTO_SUB_MODES = {1: "READY", 2: "TAKEOFF", 3: "LOITER", 4: "MISSION",
                      5: "RTL", 6: "LAND", 7: "RTGS", 8: "FOLLOWME",
                      9: "PRECLAND"}


def _mode_string(msg):
    """HEARTBEAT -> flight mode name.

    pymavlink's decoder returns "UNKNOWN" whenever base_mode does not carry the
    flag combination it expects, which happens often enough that the UI would
    just show UNKNOWN. Fall back to decoding custom_mode ourselves.
    """
    try:
        name = mavutil.mode_string_v10(msg)
    except Exception:                                          # noqa: BLE001
        name = None
    if name and name != "UNKNOWN":
        return name
    custom = int(getattr(msg, "custom_mode", 0) or 0)
    main, sub = (custom >> 16) & 0xFF, (custom >> 24) & 0xFF
    if main == 4 and sub in PX4_AUTO_SUB_MODES:
        return PX4_AUTO_SUB_MODES[sub]
    if main in PX4_MAIN_MODES:
        return PX4_MAIN_MODES[main]
    return "Mode(%u)" % custom


# ----------------------------------------------------------------------------
# the link
# ----------------------------------------------------------------------------

class MavLink:
    """Owns the MAVLink connection and the telemetry state.

    ``message_handlers`` is a list of callables invoked as ``handler(msg)`` from
    the receive thread for every message, *before* this class parses it. A
    handler returning True consumes the message and our own parsing is skipped.
    ``log_download.LogDownloader.handle`` is exactly that.
    """

    def __init__(self, address="auto", baud=57600, message_handlers=None):
        self._address = address or "auto"
        self._baud = int(baud)
        self._handlers = list(message_handlers or [])

        self._lock = threading.Lock()          # guards all telemetry below
        self._send_lock = threading.Lock()     # hold to touch master.mav.*_send
        self._stop = threading.Event()
        self._cmd_q = queue.Queue()
        self._thread = None
        self._master = None

        # link
        self._resolved = self._address
        self._link_up = False
        self._packets = 0
        self._packets_at_open = -1     # -1 so the very first open is logged
        self._drops = 0
        self._drops_base = 0
        self._rx_t = 0.0
        self._error = ""
        self._error_printed = {}   # message -> last printed monotonic time

        # heartbeat / arming (authoritative: the vehicle's own HEARTBEAT)
        self._armed = False
        self._hb_t = 0.0
        self._mode = ""

        # attitude
        self._att = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                     "rollspeed": 0.0, "pitchspeed": 0.0, "yawspeed": 0.0}
        self._att_t = 0.0

        # local position / velocity (NED)
        self._pos = {"x": 0.0, "y": 0.0, "z": 0.0,
                     "vx": 0.0, "vy": 0.0, "vz": 0.0}
        self._pos_t = 0.0

        # height AGL
        self._agl = float("nan")
        self._agl_valid = False
        self._agl_t = 0.0

        # battery
        self._batt = {"voltage": float("nan"), "current": float("nan"),
                      "remaining": float("nan")}

        # IR beacon
        self._ir = {"valid": False, "dx_px": 0.0, "dy_px": 0.0,
                    "angle_x": float("nan"), "angle_y": float("nan"),
                    "spot_id": 0, "spot_score": 0, "via": "DEBUG_FLOAT_ARRAY"}
        self._ir_t = 0.0
        self._ir_count = 0

        self._arrays = {}                      # name -> [float, ...]
        self._params = {}                      # name -> float
        self._param_types = {}                 # name -> MAV_PARAM_TYPE_*
        self._messages = deque(maxlen=MAX_MESSAGES)
        self._ack = None
        self._param_list_t = 0.0

    # -- public API ---------------------------------------------------------

    @property
    def master(self):
        """The pymavlink connection, or None while disconnected.

        Hold :attr:`send_lock` while sending on it. Never call blocking
        receives on it from outside the receive thread.
        """
        return self._master

    @property
    def send_lock(self):
        return self._send_lock

    @property
    def address(self):
        return self._resolved

    def start(self):
        """Spawn the receive thread. Returns immediately."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="mavlink-rx",
                                        daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        th = self._thread
        if th is not None:
            th.join(timeout=3.0)
        self._thread = None
        self._close()

    def arm(self, force=False):
        """Queue an arm request. Never blocks, never touches the link."""
        self._cmd_q.put(("arm", bool(force)))

    def disarm(self, force=False):
        self._cmd_q.put(("disarm", bool(force)))

    def set_param(self, name, value):
        """Queue a PARAM_SET.

        No arm gate here on purpose: the contract puts that check in
        ``server.py`` (``/api/param`` refuses while armed unless the request
        carries ``force``), and duplicating it here with a different signature
        would make the documented HTTP behaviour unreachable.
        """
        self._cmd_q.put(("param_set", str(name), float(value)))

    def request_params(self, prefix=""):
        """Queue a PARAM_REQUEST_LIST.

        MAVLink has no server-side prefix filter, so we always fetch the whole
        set into ``state["params"]`` and let the caller filter. ``prefix`` is
        accepted for contract compatibility and only used for logging.
        """
        self._cmd_q.put(("param_list", str(prefix or "")))

    def snapshot(self):
        """Thread-safe, JSON-safe copy of STATE (see WEBUI_CONTRACT.md)."""
        now = time.monotonic()
        with self._lock:
            link_up = self._link_up
            resolved = self._resolved
            packets, drops = self._packets, self._drops
            rx_t, error = self._rx_t, self._error
            armed, hb_t, mode = self._armed, self._hb_t, self._mode
            att, att_t = dict(self._att), self._att_t
            pos, pos_t = dict(self._pos), self._pos_t
            agl, agl_valid, agl_t = self._agl, self._agl_valid, self._agl_t
            batt = dict(self._batt)
            ir, ir_t = dict(self._ir), self._ir_t
            arrays = {k: list(v) for k, v in self._arrays.items()}
            params = dict(self._params)
            messages = [dict(m) for m in self._messages]
            ack = dict(self._ack) if self._ack else None

        connected = bool(link_up and hb_t and (now - hb_t) < LINK_TIMEOUT)

        # body-frame velocity: NED rotated by yaw
        yaw = att["yaw"]
        cy, sy = math.cos(yaw), math.sin(yaw)
        vx, vy = pos["vx"], pos["vy"]
        body_vel = {"fwd": vx * cy + vy * sy, "right": -vx * sy + vy * cy}

        # beacon ground offset, only meaningful with a real height and angle
        height_ok = bool(agl_valid) and _finite(agl) and agl > 0.0
        ir_out = {
            "valid": bool(ir["valid"]),
            "dx_px": ir["dx_px"],
            "dy_px": ir["dy_px"],
            "angle_x": ir["angle_x"],
            "angle_y": ir["angle_y"],
            "offset_fwd": _offset(agl, ir["angle_x"]) if height_ok else None,
            "offset_right": _offset(agl, ir["angle_y"]) if height_ok else None,
            "spot_id": ir["spot_id"],
            "spot_score": ir["spot_score"],
            "age": _age(ir_t, now),
            "via": ir["via"],
        }

        att_out = dict(att)
        att_out["age"] = _age(att_t, now)
        pos_out = dict(pos)
        pos_out["age"] = _age(pos_t, now)

        state = {
            "t": now,
            "connected": connected,
            "address": resolved,
            "link": {"packets": packets, "drops": drops,
                     "last_rx_age": _age(rx_t, now), "error": error},
            "armed": bool(armed),
            "arm_state_age": _age(hb_t, now),
            "mode": mode,
            "attitude": att_out,
            "position": pos_out,
            "body_vel": body_vel,
            "height": {"agl": agl, "valid": bool(agl_valid),
                       "age": _age(agl_t, now)},
            "battery": batt,
            "ir": ir_out,
            "debug_arrays": arrays,
            "params": params,
            "messages": messages,
            "ack": ack,
        }
        return json_safe(state)

    # -- receive thread ----------------------------------------------------

    def _run(self):
        attempt = 0
        while not self._stop.is_set():
            if self._master is None:
                if not self._connect():
                    delay = RECONNECT_BACKOFF[min(attempt, len(RECONNECT_BACKOFF) - 1)]
                    attempt += 1
                    self._stop.wait(delay)
                    continue
                attempt = 0
            try:
                self._pump()
            except Exception as exc:                           # noqa: BLE001
                self._note_error("link lost: %s" % exc)
                self._close()
                self._stop.wait(0.2)
        self._close()

    def _connect(self):
        address = self._address
        if address in (None, "", "auto"):
            address = autodetect_port()
            if not address:
                self._note_error("no USB device found (tried /dev/cu.usbmodem*, "
                                 "/dev/ttyACM*); pass --connect")
                return False
        serial = not is_network_address(address)
        try:
            if serial:
                master = mavutil.mavlink_connection(
                    address, baud=self._baud, source_system=255,
                    source_component=mavlink.MAV_COMP_ID_MISSIONPLANNER)
            else:
                master = mavutil.mavlink_connection(
                    address, source_system=255,
                    source_component=mavlink.MAV_COMP_ID_MISSIONPLANNER)
        except Exception as exc:                               # noqa: BLE001
            self._note_error("connect %s failed: %s" % (address, exc))
            return False
        self._master = master
        with self._lock:
            self._resolved = address
            self._link_up = True
            self._drops_base = self._drops
            # Keep the previous error visible: reopening a serial port or a
            # socket proves nothing, only received traffic does (cleared in
            # _on_message). Otherwise the UI says "not connected" with no reason.
            first_open = self._packets != self._packets_at_open
            self._packets_at_open = self._packets
        if first_open:
            # Don't log every retry of a flapping link; that would push the
            # vehicle's STATUSTEXTs out of the 100-entry ring.
            self._log(6, "link open: %s" % address)
        return True

    def _close(self):
        master, self._master = self._master, None
        with self._lock:
            self._link_up = False
        if master is not None:
            try:
                master.close()
            except Exception:                                  # noqa: BLE001
                pass

    def _pump(self):
        """One pass of the receive loop. Raises if the link dies."""
        master = self._master
        last_hb = last_streams = 0.0
        connect_t = time.monotonic()
        teardown_on_silence = not is_listening_address(self._resolved)

        while not self._stop.is_set() and self._master is master:
            now = time.monotonic()

            if now - last_hb > HEARTBEAT_INTERVAL:
                last_hb = now
                self._send(lambda: master.mav.heartbeat_send(
                    mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0))
            if now - last_streams > STREAM_INTERVAL and master.target_system:
                last_streams = now
                self._request_streams(master)

            self._drain_commands(master, now)

            # A dead USB link raises; a dead UDP peer just goes quiet, so the
            # silence check is what makes a MAVProxy restart recoverable.
            with self._lock:
                rx_t = self._rx_t
            quiet_since = max(rx_t, connect_t)
            if teardown_on_silence and now - quiet_since > SILENCE_REOPEN:
                raise IOError("no MAVLink traffic for %.1fs" % (now - quiet_since))

            msg = master.recv_match(blocking=True, timeout=0.05)
            if msg is None:
                continue
            if msg.get_type() == "BAD_DATA":
                continue
            self._on_message(master, msg, time.monotonic())

    def _on_message(self, master, msg, now):
        with self._lock:
            self._packets += 1
            self._rx_t = now
            self._drops = self._drops_base + int(getattr(master, "mav_loss", 0) or 0)
            if self._error:
                self._error = ""      # traffic is the only proof the link works

        for handler in self._handlers:
            try:
                if handler(msg):
                    return
            except Exception as exc:                           # noqa: BLE001
                self._note_error("handler %r: %s" % (handler, exc))

        self._parse(master, msg, now)

    def _parse(self, master, msg, now):
        mtype = msg.get_type()

        if mtype == "DEBUG_FLOAT_ARRAY":
            self._feed_debug_array(msg, now)

        elif mtype == "ATTITUDE":
            with self._lock:
                self._att.update(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw,
                                 rollspeed=msg.rollspeed,
                                 pitchspeed=msg.pitchspeed,
                                 yawspeed=msg.yawspeed)
                self._att_t = now

        elif mtype == "LOCAL_POSITION_NED":
            with self._lock:
                self._pos.update(x=msg.x, y=msg.y, z=msg.z,
                                 vx=msg.vx, vy=msg.vy, vz=msg.vz)
                self._pos_t = now

        elif mtype == "DISTANCE_SENSOR":
            d_cm = msg.current_distance
            lo, hi = msg.min_distance, msg.max_distance
            valid = (d_cm > 0 and (lo <= 0 or d_cm >= lo) and (hi <= 0 or d_cm <= hi))
            with self._lock:
                self._agl = d_cm / 100.0 if d_cm > 0 else float("nan")
                self._agl_valid = bool(valid)
                self._agl_t = now

        elif mtype == "HEARTBEAT":
            # Only the autopilot's own heartbeat carries the arm state; our GCS
            # heartbeat (and any other GCS on the network) must be ignored or
            # the ARM button would lie.
            if master is not None and msg.get_srcSystem() == master.target_system \
                    and msg.type != mavlink.MAV_TYPE_GCS:
                with self._lock:
                    self._armed = bool(msg.base_mode
                                       & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self._mode = _mode_string(msg)
                    self._hb_t = now

        elif mtype == "SYS_STATUS":
            v = msg.voltage_battery
            a = msg.current_battery
            pct = msg.battery_remaining
            with self._lock:
                self._batt["voltage"] = v / 1000.0 if v not in (0, 65535) else float("nan")
                self._batt["current"] = a / 100.0 if a != -1 else float("nan")
                self._batt["remaining"] = float(pct) if pct >= 0 else float("nan")

        elif mtype == "BATTERY_STATUS":
            volts = list(getattr(msg, "voltages", []) or [])
            v = volts[0] if volts else 65535
            a = msg.current_battery
            pct = msg.battery_remaining
            with self._lock:
                if v not in (0, 65535):
                    self._batt["voltage"] = v / 1000.0
                if a != -1:
                    self._batt["current"] = a / 100.0
                if pct >= 0:
                    self._batt["remaining"] = float(pct)

        elif mtype == "VFR_HUD":
            pass   # requested for rate/alt context; nothing in the schema yet

        elif mtype == "STATUSTEXT":
            self._log(int(getattr(msg, "severity", 6)), as_text(msg.text), now)

        elif mtype == "COMMAND_ACK":
            with self._lock:
                self._ack = {"cmd": int(msg.command),
                             "result": ACK_RESULT.get(int(msg.result),
                                                      "result %d" % msg.result),
                             "t": now}

        elif mtype == "PARAM_VALUE":
            name = as_text(msg.param_id)
            if name:
                ptype = int(getattr(msg, "param_type", mavlink.MAV_PARAM_TYPE_REAL32))
                with self._lock:
                    self._param_types[name] = ptype
                    self._params[name] = decode_param_value(msg.param_value, ptype)

    def _feed_debug_array(self, msg, now):
        """Generic, name-keyed DEBUG_FLOAT_ARRAY handling.

        Every array is exposed raw under its own name; IRCAM is additionally
        decoded into ``state["ir"]`` using the layout fixed by
        IrCam::publishDebugArray():
            0 dx_px 1 dy_px 2 angle_x 3 angle_y 4 valid 5 spot_id 6 spot_score
        """
        name = array_name(msg) or "?"
        vals = trim_array(getattr(msg, "data", []) or [])
        with self._lock:
            self._arrays[name] = vals
        if name != IRCAM_ARRAY_NAME:
            return False
        d = list(vals) + [0.0] * 7
        with self._lock:
            self._ir.update(dx_px=d[0], dy_px=d[1], angle_x=d[2], angle_y=d[3],
                            valid=bool(d[4] > 0.5), spot_id=safe_int(d[5]),
                            spot_score=safe_int(d[6]), via="DEBUG_FLOAT_ARRAY")
            self._ir_t = now
            self._ir_count += 1
        return True

    # -- sending -----------------------------------------------------------

    def _send(self, fn):
        """Run a send under the shared lock. Never nest this."""
        with self._send_lock:
            fn()

    def _request_streams(self, master):
        ts, tc = master.target_system, master.target_component
        for msg_id, hz in STREAMS:
            interval_us = int(1e6 / hz)
            self._send(lambda i=msg_id, u=interval_us: master.mav.command_long_send(
                ts, tc, mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                i, u, 0, 0, 0, 0, 0))

    def _drain_commands(self, master, now):
        while True:
            try:
                item = self._cmd_q.get_nowait()
            except queue.Empty:
                return
            kind = item[0]
            if not master.target_system:
                self._log(4, "no vehicle heartbeat yet, dropped %s" % kind, now)
                continue
            try:
                if kind in ("arm", "disarm"):
                    self._do_arm(master, kind == "arm", item[1], now)
                elif kind == "param_set":
                    self._do_param_set(master, item[1], item[2], now)
                elif kind == "param_list":
                    self._do_param_list(master, item[1], now)
            except Exception as exc:                           # noqa: BLE001
                self._note_error("send %s failed: %s" % (kind, exc))
                self._log(3, "send %s failed: %s" % (kind, exc), now)

    def _do_arm(self, master, want_arm, force, now):
        ts, tc = master.target_system, master.target_component
        with self._lock:
            self._ack = {"cmd": mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                         "result": "SENDING %s" % ("ARM" if want_arm else "DISARM"),
                         "t": now}
        self._send(lambda: master.mav.command_long_send(
            ts, tc, mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1.0 if want_arm else 0.0, 21196.0 if force else 0.0, 0, 0, 0, 0, 0))

    def _do_param_set(self, master, name, value, now):
        ts, tc = master.target_system, master.target_component
        with self._lock:
            ptype = self._param_types.get(name, mavlink.MAV_PARAM_TYPE_REAL32)
        wire = encode_param_value(value, ptype)
        self._send(lambda: master.mav.param_set_send(
            ts, tc, name.encode("ascii", "replace")[:16], wire, ptype))
        self._log(6, "set %s = %g" % (name, value), now)
        # PX4 echoes a PARAM_VALUE; ask for it explicitly in case it does not.
        self._send(lambda: master.mav.param_request_read_send(
            ts, tc, name.encode("ascii", "replace")[:16], -1))

    def _do_param_list(self, master, prefix, now):
        if now - self._param_list_t < PARAM_LIST_MIN_GAP:
            return
        self._param_list_t = now
        ts, tc = master.target_system, master.target_component
        self._send(lambda: master.mav.param_request_list_send(ts, tc))
        self._log(6, "requested parameter list%s"
                  % (" (filter %s)" % prefix if prefix else ""), now)

    # -- bookkeeping -------------------------------------------------------

    def _log(self, severity, text, now=None):
        entry = {"t": now if now is not None else time.monotonic(),
                 "severity": int(severity), "text": str(text)}
        with self._lock:
            self._messages.append(entry)

    def _note_error(self, text):
        text = str(text)
        with self._lock:
            self._error = text

        # The reconnect loop retries forever, so printing every attempt buries
        # everything else -- including the startup manual -- in identical lines.
        # Print a given message once, then at most every 30 s while it persists.
        # The full text is always live in state["link"]["error"] for the UI.
        now = time.monotonic()
        last_t = self._error_printed.get(text, 0.0)

        if now - last_t < 30.0:
            return

        self._error_printed[text] = now
        repeat = "" if last_t == 0.0 else "  (still, repeating every 30s)"
        print("mav_link: %s%s" % (text, repeat), file=sys.stderr)

        if len(self._error_printed) > 32:
            self._error_printed.clear()


def _finite(v):
    try:
        return math.isfinite(float(v))
    except (TypeError, ValueError):
        return False


def _age(t, now):
    if not t:
        return AGE_NEVER
    return min(max(now - t, 0.0), AGE_NEVER)


def _offset(agl, angle):
    """height * tan(angle), or None when either input is unusable."""
    if not _finite(angle) or not _finite(agl):
        return None
    if abs(angle) > math.pi / 2 - 1e-3:      # tan blows up at the horizon
        return None
    return agl * math.tan(angle)


# ----------------------------------------------------------------------------
# selftest -- runs with no hardware
# ----------------------------------------------------------------------------

class _Check:
    def __init__(self):
        self.ok = True

    def __call__(self, name, actual, want, tol=None):
        if tol is None:
            good = actual == want or (actual is None and want is None)
        else:
            try:
                good = actual is not None and abs(float(actual) - float(want)) <= tol
            except (TypeError, ValueError):
                good = False
        self.ok &= bool(good)
        print("  %s %s: %r (want %r%s)"
              % ("ok  " if good else "FAIL", name, actual, want,
                 "" if tol is None else " +/-%g" % tol))
        return good

    def truth(self, name, cond):
        self.ok &= bool(cond)
        print("  %s %s" % ("ok  " if cond else "FAIL", name))
        return bool(cond)


def _blank_link():
    """A MavLink that has never connected -- safe to poke at directly."""
    return MavLink("udpin:0.0.0.0:14550")


def selftest():
    ck = _Check()

    print("json_safe: non-finite floats -> None (recursive)")
    nan, inf = float("nan"), float("inf")
    raw = {"a": nan, "b": inf, "c": -inf, "d": 1.5, "e": 0.0,
           "list": [nan, 2.0, [inf, {"deep": -inf}]],
           "keepers": {"true": True, "int": 7, "str": "nan", "none": None},
           "tuple": (nan, 3.0)}
    out = json_safe(raw)
    ck("a NaN", out["a"], None)
    ck("b +inf", out["b"], None)
    ck("c -inf", out["c"], None)
    ck("d finite float kept", out["d"], 1.5)
    ck("e zero kept", out["e"], 0.0)
    ck("nested list NaN", out["list"][0], None)
    ck("nested list value", out["list"][1], 2.0)
    ck("2-deep list inf", out["list"][2][0], None)
    ck("3-deep dict -inf", out["list"][2][1]["deep"], None)
    ck("bool untouched", out["keepers"]["true"], True)
    ck("int untouched", out["keepers"]["int"], 7)
    ck('string "nan" untouched', out["keepers"]["str"], "nan")
    ck("None untouched", out["keepers"]["none"], None)
    ck("tuple -> list, NaN -> None", out["tuple"], [None, 3.0])
    try:
        json.dumps(out, allow_nan=False)
        ck.truth("json.dumps(allow_nan=False) accepts it", True)
    except ValueError as exc:
        ck.truth("json.dumps(allow_nan=False) accepts it (%s)" % exc, False)

    print("\nbody_vel: NED velocity rotated by yaw")
    cases = [
        # yaw deg, vx(N), vy(E), fwd, right
        ("yaw 0, due north", 0.0, 1.0, 0.0, 1.0, 0.0),
        ("yaw 90, due north", 90.0, 1.0, 0.0, 0.0, -1.0),
        ("yaw 90, due east", 90.0, 0.0, 1.0, 1.0, 0.0),
        ("yaw 180, due north", 180.0, 1.0, 0.0, -1.0, 0.0),
        ("yaw 45, NE 1,1", 45.0, 1.0, 1.0, 1.4142135624, 0.0),
        ("yaw -90, due north", -90.0, 1.0, 0.0, 0.0, 1.0),
        ("yaw 30, 2,-1", 30.0, 2.0, -1.0, 1.2320508076, -1.8660254038),
    ]
    for name, yaw_deg, vx, vy, want_f, want_r in cases:
        link = _blank_link()
        now = time.monotonic()
        link._att.update(yaw=math.radians(yaw_deg))
        link._att_t = now
        link._pos.update(vx=vx, vy=vy)
        link._pos_t = now
        s = link.snapshot()
        ck("%s fwd" % name, s["body_vel"]["fwd"], want_f, 1e-9)
        ck("%s right" % name, s["body_vel"]["right"], want_r, 1e-9)

    print("\nIRCAM decode from a real packed MAVLink_debug_float_array_message")
    data = [0.0] * 58
    data[0:7] = [-68.0, 287.0, 0.26907, 0.06636, 1.0, 2.0, 16.0]
    mav = mavlink.MAVLink(None)
    msg = mavlink.MAVLink_debug_float_array_message(
        time_usec=123456, name=b"IRCAM", array_id=0, data=data)
    msg.pack(mav)                          # exercise the real encoding path
    link = _blank_link()
    now = time.monotonic()
    ck.truth("recognised as IRCAM", link._feed_debug_array(msg, now))
    link._agl, link._agl_valid, link._agl_t = 2.0, True, now
    s = link.snapshot()
    ir = s["ir"]
    ck("dx_px", ir["dx_px"], -68.0)
    ck("dy_px", ir["dy_px"], 287.0)
    ck("angle_x", ir["angle_x"], 0.26907, 1e-5)
    ck("angle_y", ir["angle_y"], 0.06636, 1e-5)
    ck("valid", ir["valid"], True)
    ck("spot_id", ir["spot_id"], 2)
    ck("spot_score", ir["spot_score"], 16)
    ck("via", ir["via"], "DEBUG_FLOAT_ARRAY")
    ck.truth("ir.age fresh (<1s)", ir["age"] < 1.0)
    # offsets: height 2 m, hand-computed 2*tan(angle)
    ck("offset_fwd = 2*tan(angle_x)", ir["offset_fwd"], 2.0 * math.tan(0.26907), 1e-6)
    ck("offset_right = 2*tan(angle_y)", ir["offset_right"], 2.0 * math.tan(0.06636), 1e-6)
    ck.truth("IRCAM also raw in debug_arrays",
             s["debug_arrays"].get("IRCAM", [])[:2] == [-68.0, 287.0])
    ck.truth("trailing zeros trimmed but >=8 kept (%d)"
             % len(s["debug_arrays"]["IRCAM"]), len(s["debug_arrays"]["IRCAM"]) == 8)

    print("\nNaN angles -> null, no offsets")
    ndata = [0.0] * 58
    ndata[0:7] = [1.0, 2.0, float("nan"), float("nan"), 0.0, 1.0, 127.0]
    nmsg = mavlink.MAVLink_debug_float_array_message(
        time_usec=1, name=b"IRCAM", array_id=0, data=ndata)
    nmsg.pack(mav)
    link = _blank_link()
    link._feed_debug_array(nmsg, time.monotonic())
    link._agl, link._agl_valid, link._agl_t = 1.0, True, time.monotonic()
    s = link.snapshot()
    ck("angle_x null", s["ir"]["angle_x"], None)
    ck("angle_y null", s["ir"]["angle_y"], None)
    ck("offset_fwd null", s["ir"]["offset_fwd"], None)
    ck("offset_right null", s["ir"]["offset_right"], None)
    ck("valid false", s["ir"]["valid"], False)
    try:
        json.dumps(s, allow_nan=False)
        ck.truth("whole snapshot is strict JSON", True)
    except ValueError as exc:
        ck.truth("whole snapshot is strict JSON (%s)" % exc, False)

    print("\nunknown array name lands in debug_arrays only")
    odata = [0.0] * 58
    odata[0:4] = [9.0, 8.0, 7.0, 6.0]
    other = mavlink.MAVLink_debug_float_array_message(
        time_usec=2, name=b"SWAY", array_id=0, data=odata)
    other.pack(mav)
    link = _blank_link()
    consumed = link._feed_debug_array(other, time.monotonic())
    ck.truth("not claimed as IRCAM", consumed is False)
    s = link.snapshot()
    ck("SWAY[0:4]", s["debug_arrays"].get("SWAY", [])[:4], [9.0, 8.0, 7.0, 6.0])
    ck("SWAY length (>=8, zeros trimmed)",
       len(s["debug_arrays"].get("SWAY", [])), 8)
    ck.truth("ir untouched (never received)", s["ir"]["age"] == AGE_NEVER)
    ck.truth("ir not in debug_arrays", "IRCAM" not in s["debug_arrays"])

    print("\nno-data snapshot is still schema-shaped and JSON-safe")
    link = _blank_link()
    s = link.snapshot()
    for key in ("t", "connected", "address", "link", "armed", "arm_state_age",
                "mode", "attitude", "position", "body_vel", "height", "battery",
                "ir", "debug_arrays", "params", "messages", "ack"):
        ck.truth("key %s present" % key, key in s)
    ck("connected", s["connected"], False)
    ck("battery.voltage null", s["battery"]["voltage"], None)
    ck("height.agl null", s["height"]["agl"], None)
    ck("height.valid", s["height"]["valid"], False)
    ck("attitude.age never", s["attitude"]["age"], AGE_NEVER)
    ck("arm_state_age never", s["arm_state_age"], AGE_NEVER)
    ck("ack null", s["ack"], None)
    try:
        json.dumps(s, allow_nan=False)
        ck.truth("strict JSON", True)
    except ValueError as exc:
        ck.truth("strict JSON (%s)" % exc, False)

    print("\nSTATUSTEXT ring buffer + COMMAND_ACK decode")
    link = _blank_link()
    for i in range(150):
        link._log(6 if i % 2 else 3, "msg %d" % i, time.monotonic())
    s = link.snapshot()
    ck("ring capped at 100", len(s["messages"]), MAX_MESSAGES)
    ck("newest last", s["messages"][-1]["text"], "msg 149")
    ck("oldest dropped", s["messages"][0]["text"], "msg 50")
    ck.truth("entry shape",
             set(s["messages"][0]) == {"t", "severity", "text"})
    st = mavlink.MAVLink_statustext_message(severity=2, text=b"critical thing")
    st.pack(mav)
    link._parse(None, st, time.monotonic())
    s = link.snapshot()
    ck("STATUSTEXT text", s["messages"][-1]["text"], "critical thing")
    ck("STATUSTEXT severity", s["messages"][-1]["severity"], 2)
    for code, want in ACK_RESULT.items():
        ack = mavlink.MAVLink_command_ack_message(command=400, result=code)
        ack.pack(mav)
        link._parse(None, ack, time.monotonic())
        ck("ack result %d" % code, link.snapshot()["ack"]["result"], want)
    ack = mavlink.MAVLink_command_ack_message(command=400, result=99)
    ack.pack(mav)
    link._parse(None, ack, time.monotonic())
    ck("unknown ack code", link.snapshot()["ack"]["result"], "result 99")
    ck("ack cmd", link.snapshot()["ack"]["cmd"], 400)

    print("\nPX4 bytewise int parameters (DF_SWAY_EN=1 must not read as 1.4e-45)")
    pv = mavlink.MAVLink_param_value_message(
        param_id=b"DF_SWAY_EN", param_value=encode_param_value(1, mavlink.MAV_PARAM_TYPE_INT32),
        param_type=mavlink.MAV_PARAM_TYPE_INT32, param_count=1, param_index=0)
    pv.pack(mav)
    link = _blank_link()
    link._parse(None, pv, time.monotonic())
    ck("DF_SWAY_EN", link.snapshot()["params"].get("DF_SWAY_EN"), 1.0)
    pf = mavlink.MAVLink_param_value_message(
        param_id=b"DF_IRC_ROT", param_value=1.5,
        param_type=mavlink.MAV_PARAM_TYPE_REAL32, param_count=1, param_index=1)
    pf.pack(mav)
    link._parse(None, pf, time.monotonic())
    ck("DF_IRC_ROT float", link.snapshot()["params"].get("DF_IRC_ROT"), 1.5)
    ck("int round-trip 7",
       decode_param_value(encode_param_value(7, mavlink.MAV_PARAM_TYPE_INT32),
                          mavlink.MAV_PARAM_TYPE_INT32), 7.0)

    print("\nmessage_handlers hook short-circuits our parsing")
    seen = []
    link = MavLink("udpin:0.0.0.0:14550",
                   message_handlers=[lambda m: seen.append(m.get_type()) or
                                     m.get_type() == "DEBUG_FLOAT_ARRAY"])
    link._on_message(None, msg, time.monotonic())      # the IRCAM message
    ck("handler saw the message", seen, ["DEBUG_FLOAT_ARRAY"])
    ck.truth("consumed -> ir untouched",
             link.snapshot()["ir"]["age"] == AGE_NEVER)
    link2 = MavLink("udpin:0.0.0.0:14550", message_handlers=[lambda m: False])
    link2._on_message(None, msg, time.monotonic())
    ck.truth("handler returning False -> we still parse",
             link2.snapshot()["ir"]["dx_px"] == -68.0)
    ck("packets counted", link2.snapshot()["link"]["packets"], 1)

    print("\nHEARTBEAT: arm flag only from the vehicle, mode decoded")

    class _FakeMaster:
        target_system = 1
        target_component = 1
        mav_loss = 0

    fake = _FakeMaster()

    def _hb(sysid, mav_type, base_mode, custom_mode, autopilot=None):
        if autopilot is None:
            autopilot = mavlink.MAV_AUTOPILOT_PX4
        h = mavlink.MAVLink_heartbeat_message(mav_type, autopilot, base_mode,
                                              custom_mode, 3, 3)
        mav.srcSystem, mav.srcComponent = sysid, 1
        mav.seq = 0
        h.pack(mav)
        h = mavlink.MAVLink(None).decode(h.get_msgbuf())
        return h

    armed_custom = (3 << 16)      # PX4 main mode 3 = POSCTL
    base_armed = (mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                  | mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
                  | mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    link = _blank_link()
    link._parse(fake, _hb(1, mavlink.MAV_TYPE_QUADROTOR, base_armed, armed_custom),
                time.monotonic())
    s = link.snapshot()
    ck("vehicle armed flag", s["armed"], True)
    ck("mode POSCTL", s["mode"], "POSCTL")
    # our own GCS heartbeat must never move the arm flag back
    link._parse(fake, _hb(1, mavlink.MAV_TYPE_GCS, 0, 0,
                          mavlink.MAV_AUTOPILOT_INVALID), time.monotonic())
    ck("GCS heartbeat ignored", link.snapshot()["armed"], True)
    # nor may another system's heartbeat
    link._parse(fake, _hb(7, mavlink.MAV_TYPE_QUADROTOR,
                          mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0),
                time.monotonic())
    ck("foreign sysid ignored", link.snapshot()["armed"], True)
    link._parse(fake, _hb(1, mavlink.MAV_TYPE_QUADROTOR,
                          mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                          | mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
                          (2 << 16)), time.monotonic())
    s = link.snapshot()
    ck("disarm seen", s["armed"], False)
    ck("mode ALTCTL", s["mode"], "ALTCTL")
    # base_mode without the flags pymavlink expects -> our own fallback decode
    link._parse(fake, _hb(1, mavlink.MAV_TYPE_QUADROTOR,
                          mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                          (3 << 16)), time.monotonic())
    ck("mode fallback POSCTL", link.snapshot()["mode"], "POSCTL")
    link._parse(fake, _hb(1, mavlink.MAV_TYPE_QUADROTOR,
                          mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                          (4 << 16) | (6 << 24)), time.monotonic())
    ck("mode fallback AUTO.LAND", link.snapshot()["mode"], "LAND")
    ck.truth("arm_state_age fresh", link.snapshot()["arm_state_age"] < 1.0)

    print("\nage helpers and offsets")
    ck("_age never", _age(0.0, 100.0), AGE_NEVER)
    ck("_age normal", _age(99.5, 100.0), 0.5, 1e-9)
    ck("_age clamped", _age(1.0, 1e7), AGE_NEVER)
    ck("_offset(2, atan(0.5))", _offset(2.0, math.atan(0.5)), 1.0, 1e-9)
    ck("_offset NaN angle", _offset(2.0, float("nan")), None)
    ck("_offset NaN height", _offset(float("nan"), 0.1), None)
    ck("_offset near horizon", _offset(2.0, math.pi / 2), None)
    ck("array_name from bytes", array_name(msg), "IRCAM")

    class _FakeName:
        name = "IRCAM\x00\x00\x00\x00\x00"
    ck("array_name from padded str", array_name(_FakeName()), "IRCAM")

    class _FakeList:
        name = [73, 82, 67, 65, 77, 0, 0, 0, 0, 0]
    ck("array_name from int list", array_name(_FakeList()), "IRCAM")
    ck("trim keeps 8 minimum", len(trim_array([1.0] + [0.0] * 50)), 8)
    ck("trim keeps trailing data",
       trim_array([0.0] * 9 + [5.0] + [0.0] * 10), [0.0] * 9 + [5.0])

    print("\nPASS" if ck.ok else "\nFAIL")
    return 0 if ck.ok else 1


# ----------------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--connect", default="auto",
                   help="auto | /dev/cu.usbmodem01 | udpin:0.0.0.0:14550 | "
                        "udpout:host:port | udp:host:port | tcp:host:port")
    p.add_argument("--baud", type=int, default=57600)
    p.add_argument("--dump", action="store_true",
                   help="connect and print a STATE snapshot every second")
    p.add_argument("--selftest", action="store_true",
                   help="run offline checks and exit")
    args = p.parse_args()

    if args.selftest:
        return selftest()
    if not args.dump:
        p.print_help()
        return 0

    link = MavLink(args.connect, baud=args.baud)
    link.start()
    try:
        while True:
            time.sleep(1.0)
            print(json.dumps(link.snapshot(), allow_nan=False, sort_keys=True))
    except KeyboardInterrupt:
        pass
    finally:
        link.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
