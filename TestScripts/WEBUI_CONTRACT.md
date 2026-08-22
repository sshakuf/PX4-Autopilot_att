# webui — interface contract

Browser-based controller/telemetry UI for the rope-hung horizontal drone.
Replaces the matplotlib-only `ir_flow_monitor.py` with something reachable from a
phone.

**This file is the contract between independently-written modules. Do not change
a shared name or shape without updating it here.**

## Hard constraints

- **Dependencies: Python standard library + `pymavlink` only.** No Flask,
  FastAPI, aiohttp, websockets, numpy or pandas. Telemetry push uses
  Server-Sent Events (SSE), which is plain HTTP and works in iOS Safari.
- **MAVLink 2 dialect required.** Set before importing pymavlink:
  ```python
  os.environ.setdefault("MAVLINK20", "1")
  os.environ.setdefault("MAVLINK_DIALECT", "common")
  ```
  `DEBUG_FLOAT_ARRAY` is message id 350 and does not exist in MAVLink 1.
- **Python 3.9+**, must run on macOS and on a Raspberry Pi.
- Never block the HTTP thread on MAVLink I/O. One receive thread owns the link;
  HTTP handlers only read a snapshot or post to a queue.

## What is and is not reachable over MAVLink

Reachable (use these): `HEARTBEAT`, `ATTITUDE`, `LOCAL_POSITION_NED`,
`GLOBAL_POSITION_INT`, `SYS_STATUS`, `BATTERY_STATUS`, `DISTANCE_SENSOR`,
`VFR_HUD`, `STATUSTEXT`, `PARAM_VALUE`, `EXTENDED_SYS_STATE`,
`ESTIMATOR_STATUS`, `DEBUG_FLOAT_ARRAY`.

**NOT reachable:** `swing_damper_status`, `pos_control_health`,
`target_hold_status`, `ir_camera_report`. These are custom uORB topics; no
MAVLink client can subscribe to them. Only `ir_camera_report` currently has a
bridge, mirrored into `debug_array` named `IRCAM` by the
`IR_CAM_DEBUG_MAVLINK` block in `IrCam.cpp`.

Therefore `DEBUG_FLOAT_ARRAY` handling MUST be **generic and name-keyed**, so
additional firmware bridges appear without frontend changes. Known layouts:

| name | index → field |
|---|---|
| `IRCAM` | 0 dx_px, 1 dy_px, 2 angle_x, 3 angle_y, 4 valid, 5 spot_id, 6 spot_score |

Unknown names are still exposed as a raw float list under their name.

## Layout

```
TestScripts/webui/
  server.py         HTTP + SSE + routing + static serving        (owner: server)
  mav_link.py       link, telemetry state, commands, params      (owner: link)
  log_download.py   MAVLink log list + download                 (owner: logs)
  static/index.html mobile-first UI                             (owner: frontend)
  static/app.js
  static/style.css
  README.md
```

## `mav_link.py` public API

```python
class MavLink:
    def __init__(self, address: str, baud: int = 57600) -> None
    def start(self) -> None            # spawn receive thread, non-blocking
    def stop(self) -> None
    def snapshot(self) -> dict         # thread-safe copy of STATE (schema below)
    def arm(self, force: bool = False) -> None      # queued, never blocks
    def disarm(self, force: bool = False) -> None
    def set_param(self, name: str, value: float) -> None
    def request_params(self, prefix: str = "") -> None
    @property
    def master(self)                   # pymavlink connection, for log_download
    @property
    def send_lock(self)                # threading.Lock; hold to send
```

Address forms that must work:
- `auto` — glob `/dev/cu.usbmodem*`, `/dev/tty.usbmodem*`, `/dev/ttyACM*`
- `/dev/cu.usbmodem01` — serial, apply `baud`
- `udpin:0.0.0.0:14550` — listen; **this is the MAVProxy case**
- `udpout:192.168.1.50:14550`, `udp:host:port`, `tcp:host:port`

MAVProxy note: it must be started with an output aimed at this host, e.g.
`mavproxy.py --master=/dev/ttyAMA0 --out=udp:<laptop-ip>:14550`, and the server
then uses `udpin:0.0.0.0:14550`.

Request these streams via `MAV_CMD_SET_MESSAGE_INTERVAL`, re-sent every 5 s:
`ATTITUDE` 20 Hz, `LOCAL_POSITION_NED` 20 Hz, `DISTANCE_SENSOR` 10 Hz,
`DEBUG_FLOAT_ARRAY` 30 Hz, `SYS_STATUS` 2 Hz, `BATTERY_STATUS` 2 Hz,
`VFR_HUD` 5 Hz, `ESTIMATOR_STATUS` 2 Hz. Send a GCS `HEARTBEAT` every 1 s.

### STATE schema — the single source of truth

```json
{
  "t": 1234.5,
  "connected": true,
  "address": "udpin:0.0.0.0:14550",
  "link": {"packets": 1234, "drops": 0, "last_rx_age": 0.02},
  "armed": false,
  "arm_state_age": 0.1,
  "mode": "POSCTL",
  "attitude":  {"roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                "rollspeed": 0.0, "pitchspeed": 0.0, "yawspeed": 0.0, "age": 0.02},
  "position":  {"x": 0.0, "y": 0.0, "z": 0.0,
                "vx": 0.0, "vy": 0.0, "vz": 0.0, "age": 0.05},
  "body_vel":  {"fwd": 0.0, "right": 0.0},
  "height":    {"agl": 0.0, "valid": true, "age": 0.1},
  "battery":   {"voltage": 0.0, "current": 0.0, "remaining": 0.0},
  "ir":        {"valid": false, "dx_px": 0.0, "dy_px": 0.0,
                "angle_x": null, "angle_y": null,
                "offset_fwd": null, "offset_right": null,
                "spot_id": 0, "spot_score": 0, "age": 9.9, "via": "DEBUG_FLOAT_ARRAY"},
  "debug_arrays": {"IRCAM": [0.0, 0.0]},
  "params": {"DF_SWAY_EN": 1.0},
  "messages": [{"t": 1234.0, "severity": 6, "text": "..."}],
  "ack": {"cmd": 400, "result": "ACCEPTED", "t": 1234.0}
}
```

Rules:
- `angle_x`/`angle_y` are `null` when NaN. **Never emit bare `NaN`** — it is not
  valid JSON and breaks `JSON.parse`. Convert every non-finite float to `null`.
- `*_age` are seconds since that field was last updated. The frontend greys out
  anything older than 1.0 s rather than showing stale numbers as live.
- `offset_fwd`/`offset_right` = `height.agl * tan(angle_x | angle_y)`, `null` if
  height or angle is unavailable.
- `body_vel` = NED velocity rotated by yaw: `fwd = vx·cosψ + vy·sinψ`,
  `right = −vx·sinψ + vy·cosψ`.
- `messages` is a ring buffer, newest last, max 100.

## `log_download.py` public API

```python
class LogDownloader:
    def __init__(self, link: "MavLink", out_dir: str) -> None
    def refresh(self) -> None                  # LOG_REQUEST_LIST, async
    def logs(self) -> list                     # [{id, size, utc, name, local: bool}]
    def start(self, log_id: int) -> None       # LOG_REQUEST_DATA, chunked
    def cancel(self) -> None
    def progress(self) -> dict
        # {"active": bool, "id": int, "received": int, "size": int,
        #  "pct": 0..100, "rate_bps": float, "eta_s": float, "error": str|None,
        #  "path": str|None}
    def handle(self, msg) -> bool              # called from the receive thread
```

Uses `LOG_REQUEST_LIST` / `LOG_ENTRY` / `LOG_REQUEST_DATA` / `LOG_DATA` (90-byte
chunks). Must re-request gaps, tolerate out-of-order chunks, and time out
cleanly. Save as `<out_dir>/log_<id>_<utc>.ulg`. `out_dir` defaults to
`TestScripts/webui/downloads/`.

`handle()` returns True if it consumed the message. `MavLink`'s receive loop
calls it before its own parsing.

## HTTP API — `server.py`

| method | path | body / query | returns |
|---|---|---|---|
| GET | `/` | — | `static/index.html` |
| GET | `/static/<file>` | — | static asset, correct MIME |
| GET | `/api/state` | — | STATE json |
| GET | `/api/stream` | — | SSE, `event: state`, one STATE per message, 10 Hz |
| POST | `/api/command` | `{"cmd":"arm"\|"disarm","force":bool}` | `{"ok":bool,"error":str?}` |
| GET | `/api/params` | `?prefix=DF_` | `{"params":{name:value}}` |
| POST | `/api/param` | `{"name":str,"value":float}` | `{"ok":bool}` |
| GET | `/api/logs` | — | `{"logs":[...],"progress":{...}}` |
| POST | `/api/logs/refresh` | — | `{"ok":true}` |
| POST | `/api/logs/download` | `{"id":int}` | `{"ok":bool}` |
| POST | `/api/logs/cancel` | — | `{"ok":true}` |
| GET | `/api/logs/file/<name>` | — | the `.ulg`, `Content-Disposition: attachment` |
| GET | `/api/downloads` | — | `{"files":[{"name","size","mtime"}]}` |

- `ThreadingHTTPServer`. SSE handler loops until the client disconnects; catch
  `BrokenPipeError`/`ConnectionResetError` and exit the handler quietly.
- Every response `Cache-Control: no-store` except `/static/*`.
- CLI: `--connect` (default `auto`), `--baud` (57600), `--port` (8080),
  `--host` (`0.0.0.0`, so a phone can reach it), `--out-dir`.
- On start print the LAN URL to open on the phone.

## Frontend requirements — `static/`

Mobile-first, dark, one-thumb operation on an iPhone. No frameworks, no CDN —
must work with no internet on the flying field. Vanilla JS, SSE via
`EventSource("/api/stream")`.

Panels, in this order:

1. **Link + arm bar** (sticky top): connected state, mode, packet rate. Big
   ARM/DISARM button with the **exact semantics of `ir_flow_monitor.py`**: label
   follows the real `armed` flag from HEARTBEAT, never the requested state; ARM
   and DISARM each act on a single tap (no confirmation, by request); greyed out
   and inert when `arm_state_age > 3 s`. Show the last `ack.result`.
2. **Body-frame view**: canvas, top-down, screen up = nose. Beacon dot from
   `ir.offset_fwd/right`, dashed camera FOV rectangle (half-angles
   `atan(618/1047)` lateral, `atan(480/1065)` fore/aft, scaled by
   `height.agl`), velocity arrow from `body_vel`, and a trail. Grey the beacon
   when `ir.age > 1 s`.
3. **State grid**: attitude (deg), body velocity, NED position, height, battery.
   Grey any value whose `age > 1 s`.
4. **Params**: filter box, edit-and-set, with the `DF_*` groups given quick
   buttons — `DF_SWAY_EN`, `DF_TGT_HOLD_EN`, `DF_YAW_HOLD_EN`, `DF_IRC_ROT`.
5. **Logs**: list from the vehicle, per-row download with a progress bar and
   cancel, plus a list of already-downloaded files as tappable links.
6. **Messages**: `STATUSTEXT` feed, colour-coded by severity.

Must degrade gracefully: if SSE drops, show "reconnecting" and retry with
backoff; never leave stale numbers looking live.

## Safety

- Arming spins props. The two-tap confirm and the real-state label are
  requirements, not suggestions.
- No parameter write may be sent while `armed` is true unless the request carries
  `"force": true`.
- The server binds `0.0.0.0` with **no authentication**. It is a field tool on a
  trusted network — say so in the README, do not pretend otherwise.
