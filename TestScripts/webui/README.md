# webui — browser controller + telemetry for the rope-hung drone

A single-file HTTP server (`server.py`) that puts the drone's live state, its
parameters, and its logs in a phone browser. It replaces the matplotlib-only
`../ir_flow_monitor.py`, which needed a laptop with a display attached to the
vehicle. This you can open on an iPhone while standing under the rope.

```
you ──wifi──▶ server.py ──MAVLink──▶ PX4 (micoair h743 v2)
   http/SSE      ▲
                 └─ one receive thread owns the link; HTTP handlers only read
                    a snapshot, so a slow browser can never stall telemetry
```

What it serves:

| panel | source |
|---|---|
| link state, mode, ARM/DISARM | `HEARTBEAT`, `COMMAND_ACK` |
| attitude, NED position, body velocity | `ATTITUDE`, `LOCAL_POSITION_NED` |
| height AGL | `DISTANCE_SENSOR` |
| IR beacon offset + body-frame view | `DEBUG_FLOAT_ARRAY` named `IRCAM` |
| battery | `SYS_STATUS`, `BATTERY_STATUS` |
| parameters (`DF_*` quick buttons) | `PARAM_VALUE` / `PARAM_SET` |
| `.ulg` log list + download | `LOG_ENTRY` / `LOG_DATA` |
| message feed | `STATUSTEXT` |

Custom uORB topics (`swing_damper_status`, `pos_control_health`,
`target_hold_status`, `ir_camera_report`) are **not** reachable over MAVLink.
The only one bridged today is `ir_camera_report`, mirrored into a
`DEBUG_FLOAT_ARRAY` named `IRCAM` by the `IR_CAM_DEBUG_MAVLINK` block in
`IrCam.cpp`. See `../WEBUI_CONTRACT.md`.

---

## ⚠️ Security: there is none

The server binds `0.0.0.0` and has **no authentication, no TLS, no rate
limiting**. Anybody who can reach the port can arm the vehicle, write
parameters and download logs.

It is a field tool for a trusted network — your own phone hotspot or a private
field AP. Do not put it on a shared/office/hotel network, do not port-forward
it, do not expose it to the internet. If you need to be careful, bind loopback
and tunnel:

```bash
python3 server.py --host 127.0.0.1        # then: ssh -L 8080:127.0.0.1:8080 pi@...
```

---

## Install

Standard library plus `pymavlink`. That is the whole dependency list.

```bash
cd TestScripts/webui
python3 -m pip install -r requirements.txt      # just pymavlink
```

Python 3.9+. Works on macOS and Raspberry Pi OS.

MAVLink 2 is mandatory (`DEBUG_FLOAT_ARRAY` is message id 350 and does not
exist in MAVLink 1). `server.py` sets `MAVLINK20=1` and
`MAVLINK_DIALECT=common` before pymavlink is imported, so you do not have to
export anything yourself.

---

## Run: USB straight to the flight controller

Plug the micoair into the laptop and:

```bash
python3 server.py                      # --connect auto is the default
```

`auto` globs `/dev/cu.usbmodem*`, `/dev/tty.usbmodem*`, `/dev/ttyACM*` and
takes the first hit. If you have several boards plugged in, name the device:

```bash
# macOS
python3 server.py --connect /dev/cu.usbmodem01
# Linux / Raspberry Pi
python3 server.py --connect /dev/ttyACM0
```

`--baud` (default `57600`) matters for a real UART (telemetry radio, `ttyAMA0`).
It is ignored by USB CDC-ACM, so leave it alone for a USB cable.

On start it prints every URL it can reach itself on:

```
  drone webui  —  http server on 0.0.0.0:8080
  link     : auto @ 57600 baud
  out-dir  : .../TestScripts/webui/downloads

  open one of these on your phone (same wifi):
      http://127.0.0.1:8080/
      http://192.168.0.219:8080/

  !! NO AUTHENTICATION, all interfaces. Trusted network only.
```

Open the `192.168.x.x` one on the phone. Same wifi as the laptop.

---

## Run: MAVProxy on a Raspberry Pi

The normal field setup. The Pi is on the vehicle, wired to the FC's telemetry
UART; the laptop/phone is on the ground. MAVProxy owns the serial port and
forwards MAVLink over UDP.

**On the Pi:**

```bash
mavproxy.py --master=/dev/ttyAMA0 --baudrate 921600 --out=udp:<laptop-ip>:14550
```

Notes:
- `--master` is the *serial* link to the FC. `--baudrate` must match the PX4
  `SER_TEL1_BAUD` (or whichever UART you wired). 921600 is typical for a direct
  Pi↔FC wire; a telemetry radio is usually 57600.
- `--out=udp:<laptop-ip>:14550` tells MAVProxy to **send** to your laptop. Put
  the laptop's actual LAN IP there, not `localhost`, not the Pi's IP.
- You can add more `--out=` flags to feed QGC at the same time.
- If you want the Pi itself to run the web UI, use `--out=udp:127.0.0.1:14550`
  and run `server.py` on the Pi. Then open the Pi's IP from the phone.

**On the laptop (or the Pi, per above):**

```bash
python3 server.py --connect udpin:0.0.0.0:14550
```

### `udpin` vs `udpout` — read this before you debug anything

This is the single most common reason "it connects but no data arrives".

| form | who initiates | use when |
|---|---|---|
| `udpin:0.0.0.0:14550` | **we bind and wait.** The peer sends to us; we learn its address from the first packet. | The other side was given an `--out=` / `--endpoint` aimed at us. **This is the MAVProxy case.** |
| `udpout:192.168.1.50:14550` | **we send first**, to a listener at that address. | The other side is *listening* on that port (e.g. `mavproxy.py --master=udpin:0.0.0.0:14550`, or PX4 SITL). |

Rule of thumb: **exactly one side listens.** Whoever was configured with an
"out" is the connector, so the other side must be `udpin`. Two `udpin`s means
nobody ever speaks; two `udpout`s means nobody is ever listening. Both look
identical from the outside — the UI just sits at "no link".

Also supported: `udp:host:port` (alias of `udpout`), `tcp:host:port`.

If the link stays dead:

1. Is `<laptop-ip>` in the MAVProxy `--out` actually the laptop's current IP?
   It changes when you switch networks.
2. Confirm packets are arriving at all:
   `python3 -c "import socket;s=socket.socket(2,2);s.bind(('0.0.0.0',14550));print(len(s.recv(2048)),'bytes')"`
   (stop `server.py` first — only one process can bind the port).
3. macOS firewall will silently drop inbound UDP to a new binary. Allow
   `python3` when prompted, or in System Settings → Network → Firewall.
4. MAVProxy already running and holding `/dev/ttyAMA0`? Only one process gets
   the serial port. Do not run `server.py --connect /dev/ttyAMA0` at the same
   time as MAVProxy.

---

## CLI

| flag | default | meaning |
|---|---|---|
| `--connect ADDR` | `auto` | `auto`, a serial device, `udpin:host:port`, `udpout:host:port`, `udp:host:port`, `tcp:host:port` |
| `--baud N` | `57600` | serial baud (ignored by USB CDC-ACM) |
| `--port N` | `8080` | HTTP port |
| `--host ADDR` | `0.0.0.0` | HTTP bind address. `0.0.0.0` so a phone can reach it; `127.0.0.1` to lock it down |
| `--out-dir DIR` | `./downloads` | where downloaded `.ulg` files land |

`ctrl-c` stops the server and the link.

---

## HTTP API

| method | path | body / query | returns |
|---|---|---|---|
| GET | `/` | — | `static/index.html` |
| GET | `/static/<file>` | — | static asset with correct MIME |
| GET | `/api/state` | — | STATE json |
| GET | `/api/stream` | — | SSE, `event: state`, ~10 Hz |
| POST | `/api/command` | `{"cmd":"arm"\|"disarm","force":bool}` | `{"ok":bool,"error"?:str}` |
| GET | `/api/params` | `?prefix=DF_` | `{"params":{name:value},"prefix":str}` |
| POST | `/api/param` | `{"name":str,"value":float,"force"?:bool}` | `{"ok":bool}` |
| GET | `/api/logs` | — | `{"logs":[...],"progress":{...}}` |
| POST | `/api/logs/refresh` | — | `{"ok":true}` |
| POST | `/api/logs/download` | `{"id":int}` | `{"ok":bool}` |
| POST | `/api/logs/cancel` | — | `{"ok":true}` |
| GET | `/api/logs/file/<name>` | — | the `.ulg`, `Content-Disposition: attachment` |
| GET | `/api/downloads` | — | `{"files":[{"name","size","mtime"}]}` |
| GET | `/api/health` | — | uptime, SSE client count, module import errors |

Conventions:

- Every JSON response is `Cache-Control: no-store`. `/static/*` uses
  `no-cache` so an edited `app.js` is picked up on reload.
- **No non-finite floats ever reach the wire.** `NaN` and `Infinity` are not
  valid JSON and would make `JSON.parse` throw, taking the whole UI down, so
  every non-finite value is converted to `null` before serialisation. A `null`
  `angle_x` means "the camera has no fix", not "zero".
- `*_age` fields are seconds since that value last changed. The frontend greys
  out anything older than 1.0 s instead of showing a stale number as live.
- Errors are `{"ok": false, "error": "<something readable>"}` with a real
  status code: `400` bad request, `403` forbidden path, `404` not found,
  `409` refused for safety, `500` internal, `503` link/module unavailable. No
  silent no-ops.
- A handler raising never kills the server; it returns 500 and the next request
  is served normally.

### `/api/stream` (SSE)

```
Content-Type: text/event-stream
Cache-Control: no-store
Connection: keep-alive
X-Accel-Buffering: no
```

One `event: state` frame with the full STATE object every ~100 ms, plus a
`: keepalive` comment every 15 s so idle proxies and phone NAT tables do not
drop the connection. Client disconnects (`BrokenPipeError`,
`ConnectionResetError`, socket timeout) end the handler silently — a phone
locking its screen is the normal way a stream ends and must not print a
traceback. The frontend uses `EventSource`, which reconnects on its own; the
server sends `retry: 2000` to set the backoff floor.

```bash
curl -N http://127.0.0.1:8080/api/stream          # ctrl-c to stop
```

### Static file safety

`static/` is served with `..` rejected anywhere in the (percent-decoded) path,
absolute paths and leading `/` rejected, and a final realpath check confining
the result inside `static/`. `/api/logs/file/<name>` additionally requires a
bare filename with no separators. All of these return `403`.

---

## Safety behaviour

These are requirements, not suggestions. Arming spins props while the airframe
hangs at head height on a rope.

**ARM and DISARM each take ONE tap** — there is no confirmation step. Arming spins props, so treat the button accordingly.

The button label always follows the *real* `armed` flag from the vehicle's HEARTBEAT, never what was requested, so a refused arm can never look like success. A tap with no live heartbeat (`arm_state_age > 3 s`) is ignored rather than queued, so a command cannot fire the instant a stale link returns. `force disarm (21196)` is a checkbox, off by default, and applies to disarm only.

**Parameter writes are refused while armed.** `POST /api/param` with
`armed == true` returns `409 Conflict`:

```json
{"ok": false, "armed": true,
 "error": "refused: vehicle is ARMED. Parameter writes while armed can change
           control behaviour mid-flight. Disarm first, or resend with \"force\": true."}
```

Nothing is sent to the vehicle. To override deliberately — e.g. tuning
`DF_SWAY_EN` during a tethered bench run — resend with `"force": true`. The
override is explicit and per-request; there is no sticky "unsafe mode".

Non-numeric or non-finite values are rejected with `400` before anything is
queued.

---

## Log download flow

Logs come over MAVLink in 90-byte `LOG_DATA` chunks. This is slow — expect
minutes over a 57600 telemetry radio, seconds over USB. Prefer pulling the SD
card for big logs; this is for "grab the last flight without landing the
setup".

1. `POST /api/logs/refresh` → sends `LOG_REQUEST_LIST`. Returns immediately;
   `LOG_ENTRY` messages arrive asynchronously.
2. `GET /api/logs` → `{"logs":[{id,size,utc,name,local}], "progress":{...}}`.
   Poll it (the UI does, while the list is open). `local: true` means that log
   is already in `--out-dir`.
3. `POST /api/logs/download {"id": 7}` → starts a chunked transfer.
4. `GET /api/logs` → watch `progress`:
   `{"active", "id", "received", "size", "pct", "rate_bps", "eta_s", "error", "path"}`.
   Missing chunks are re-requested and out-of-order chunks tolerated; a stalled
   transfer times out and reports `error` rather than hanging forever.
5. `POST /api/logs/cancel` → abort. Partial files are not offered as complete.
6. Finished files land in `--out-dir` as `log_<id>_<utc>.ulg`, appear in
   `GET /api/downloads`, and are downloadable to the phone from
   `GET /api/logs/file/<name>` (`Content-Disposition: attachment`, so iOS
   offers "Save to Files" instead of rendering binary).

Only one transfer runs at a time. Log traffic is consumed by the link's single
receive thread — `LogDownloader.handle` is registered through `MavLink`'s
`message_handlers` constructor argument, so a download does not need a second
reader on the socket and cannot race the telemetry parser.

---

## Layout

```
TestScripts/webui/
  server.py          HTTP + SSE + routing + static serving      (this file's owner)
  mav_link.py        link, telemetry state, commands, params
  log_download.py    MAVLink log list + download
  static/index.html  mobile-first dark UI
  static/app.js
  static/style.css
  downloads/         default --out-dir for .ulg files
  requirements.txt
  README.md
```

`server.py` degrades rather than dies: if `mav_link.py` or `log_download.py`
is missing or fails to import, the server still starts and serves the UI, and
the affected endpoints return a `503` naming the actual import error
(`GET /api/health` shows it too). `/api/state` keeps returning a full
STATE-shaped object with `connected: false` and an `error` field, so the
frontend renders "no link" instead of breaking.

The wire format between these modules is defined in `../WEBUI_CONTRACT.md`.
Do not change a shared name or shape without updating that file.

---

## Troubleshooting

| symptom | cause |
|---|---|
| UI loads, everything greyed out, `mode: NO LINK` | no MAVLink. Check `udpin`/`udpout` above and `GET /api/health` |
| `Address already in use` | another `server.py` (or a UDP listener on 14550) is running |
| phone cannot load the page | phone on cellular/another wifi; or `--host 127.0.0.1`; or laptop firewall |
| values freeze but link says connected | stream rates not applied — the server re-sends `MAV_CMD_SET_MESSAGE_INTERVAL` every 5 s; check `STATUSTEXT` for `command denied` |
| beacon dot missing but everything else live | `IRCAM` `DEBUG_FLOAT_ARRAY` absent — firmware built without `IR_CAM_DEBUG_MAVLINK`, or no spot in view (`ir.valid == false`) |
| ARM button inert | `arm_state_age > 3 s`; heartbeats stopped. Intentional |
| param write silently ignored | it is not silent — it returns `409` while armed. Check the response |
| SSE reconnect loop on iOS | phone slept; `EventSource` reconnects with backoff. Expected |
