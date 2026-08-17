# TestScripts

Bench/diagnostic tooling for the IR-beacon target-hold work. Nothing here writes
parameters. `ir_flow_monitor.py` **can arm and disarm** — see below.

## ir_flow_monitor.py

Live top-down **body-frame** view of the IR beacon and the optical-flow motion.

```
        NOSE (body +X)
             ^
             |
  LEFT <-----+-----> RIGHT (body +Y)
             |
           TAIL
```

| On screen | Meaning |
|---|---|
| red dot | beacon, projected to the ground from `angle_x`/`angle_y` × height |
| dark red line | beacon trail (last ~150 samples) |
| yellow dashed box | camera ground footprint — outside it the beacon is **not visible** |
| drifting blue dots | apparent ground motion = what the optical flow sees |
| green arrow | drone velocity, body frame, from the EKF |

Drone moves forward ⇒ dots drift **down**. If the dots move the *same* way as the
green arrow, something is sign-inverted.

### Setup

```bash
pip install -r requirements.txt
python3 ir_flow_monitor.py --selftest    # parser check, no hardware
python3 ir_flow_monitor.py --demo        # UI check, no hardware
```

### Run

**Close QGroundControl first** — the MAVLink shell is exclusive and QGC will
fight the script for it.

```bash
python3 ir_flow_monitor.py                                   # auto-detect USB
python3 ir_flow_monitor.py --connect /dev/cu.usbmodem01
python3 ir_flow_monitor.py --height 1.2 --range 2.0 --csv run1.csv
```

Keys: `r` reset displacement origin · `c` clear trail · `+`/`-` dot gain · `q` quit

### Arm / disarm button

Bottom-left. **The label always reflects the real arm state** taken from the
autopilot's `HEARTBEAT` (`base_mode & MAV_MODE_FLAG_SAFETY_ARMED`) — never from
what we last requested, so a rejected arm cannot make the button read "armed".

| Button | Meaning |
|---|---|
| green **ARM** | disarmed, link alive |
| orange **CONFIRM ARM n** | first click received, click again within 3 s |
| red **DISARM** | actually armed |
| grey **NO LINK** | no autopilot heartbeat for 3 s — clicks ignored |

**Arm needs two clicks, disarm needs one.** Deliberate asymmetry: arming spins
props so a stray click must not do it, while disarm is always the safe direction
and must never be gated.

The `COMMAND_ACK` result is decoded next to the button (`ACCEPTED`, `DENIED`,
`TEMPORARILY REJECTED`, …) so a refused arm tells you why instead of silently
doing nothing.

`--force-disarm` adds the 21196 force flag so disarm also works in flight. Off by
default.

The UI thread never touches the MAVLink link — clicks are posted to a queue and
sent by the receive thread, because pymavlink connections are not safe for
concurrent sends.

### How it gets the data

- **Motion** — ordinary MAVLink: `LOCAL_POSITION_NED`, `ATTITUDE`,
  `DISTANCE_SENSOR`, requested via `MAV_CMD_SET_MESSAGE_INTERVAL`.
- **IR target** — `ir_camera_report` is a *custom uORB topic, not a MAVLink
  message*, so nothing on the ground can subscribe to it directly. Two paths,
  selected with `--ir-source`:
  - `array` (**preferred**) — the `IR_CAM_DEBUG_MAVLINK` block in `IrCam.cpp`
    mirrors each report into `debug_array`, which PX4 streams as
    `DEBUG_FLOAT_ARRAY` named `IRCAM` at 50 Hz over USB. Needs the patched
    firmware. Full rate, no console, no QGC conflict.
  - `shell` (fallback) — drives the NuttX shell over `SERIAL_CONTROL`
    (the mechanism behind QGC's MAVLink Console), runs
    `listener ir_camera_report` and parses the text. Works on unpatched
    firmware. Slow and lossy.
  - `auto` (default) — waits `--array-wait` seconds for the array, then falls
    back to the shell. The HUD shows which one is live (`via ...`).

### IR_CAM_DEBUG_MAVLINK firmware block

Temporary diagnostic, guarded by `#define IR_CAM_DEBUG_MAVLINK 1` in
`IrCam.hpp`. Set it to `0` (or delete the two fenced blocks in `IrCam.hpp` and
`IrCam.cpp`) to remove it entirely. `data[]` layout:

| idx | field |
|---|---|
| 0 | `dx_px` |
| 1 | `dy_px` |
| 2 | `angle_x` (NaN passed through) |
| 3 | `angle_y` (NaN passed through) |
| 4 | `valid` (1/0) |
| 5 | `spot_id` |
| 6 | `spot_score` |

It deliberately uses **one `debug_array`** rather than several `DEBUG_VECT`s:
`MavlinkStreamDebugVect` does a single `_debug_sub.update()` per tick and
`debug_vect` has queue depth 1, so back-to-back publications silently drop all
but the last.

`mavsdk_read_ircam.py` is a minimal MAVSDK reader for the same stream, via the
`mavlink_direct` plugin. It exists to show the MAVSDK path works; for bench work
use `ir_flow_monitor.py`. `mavsdk_server` claims the serial port exclusively, so
the two cannot run at once.

### Known limitations

- **Console fallback throughput.** `ir_camera_report` at 13 Hz is ~4 kB/s of
  text, about 62 `SERIAL_CONTROL` packets/s. PX4's shell is built for
  interactive use, not streaming, so expect a reduced effective rate and dropped
  records. Use the `array` path for anything that matters.
- **MAVLink 2 required.** `DEBUG_FLOAT_ARRAY` is message id 350, so it does not
  exist in MAVLink 1 dialects. The script sets `MAVLINK20=1` and
  `MAVLINK_DIALECT=common` before importing pymavlink and fails loudly if that
  did not take.
- **`--height`** is only the fallback used when no `DISTANCE_SENSOR` is arriving.
  A wrong height scales the metric offsets and the FOV box; the raw angles and
  pixels in the HUD are unaffected.
- The FOV box uses `atan(cx/fx)`, `atan(cy/fy)` and ignores the `k1` distortion
  term, so it is a few percent optimistic at the corners.
- Velocity is the **EKF** estimate (flow-driven), not raw flow. That is
  deliberate: it is what the position controller actually consumes.

## Test A — is the optical flow usable?

The point of this test: the flow quantises to `0.213 × height` m/s per LSB
(≈0.11 m/s at 0.5 m). If the drone's real speed is below that, flow reads exactly
zero and the EKF dead-reckons. Verify before tuning any gains.

1. Disarmed. `logger on` in nsh if you also want a `.ulg`.
2. Start the monitor with `--csv testA.csv`.
3. Hold the drone in your hands, **low (~0.5 m)**, level, nose fixed.
4. Press `r` to zero the displacement readout, mark the floor.
5. Hold still 5 s → walk **2–3 m straight** at a brisk ~0.5 m/s → hold 5 s →
   walk back to the mark → hold 5 s.
6. `q`, then `logger off`.

Read `disp body` in the HUD against the tape measure:

| Observation | Conclusion |
|---|---|
| tracks within ~20%, correct sign | flow is healthy |
| near zero while you clearly walked | flow is blind — fix before tuning |
| correct magnitude, **wrong sign** | flow axis inverted (check `SENS_FLOW_ROT`) |
| does not return to ~0 on the way back | large drift; usable range is limited |

## Height trade-off

The two sensors want opposite things — the IR footprint grows with height, the
flow resolution degrades with it:

| Height | beacon visible (lat) | (fwd) | flow LSB |
|---|---|---|---|
| 0.5 m | ±0.30 m | ±0.23 m | 0.11 m/s |
| 0.7 m | ±0.41 m | ±0.32 m | 0.15 m/s |
| 1.2 m | ±0.71 m | ±0.54 m | 0.26 m/s |
| 1.5 m | ±0.89 m | ±0.68 m | 0.32 m/s |

Low for Test A (best flow resolution, beacon not needed). ~1.2 m for target-hold
work (usable beacon envelope). Note `DF_TGT_MAX_R = 3.0` m is unreachable at any
of these heights — the camera can never see a 3 m offset.
