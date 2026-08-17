# Direct-Mode Actuator Movement Test — Design Spec

Date: 2026-07-02
Target: SpinAir horizontal drone (MicoAir H743 v2), PX4 fork, branch `test_yaw_speed`

## Purpose

An interactive MAVLink test harness to validate that the drone's actuators
respond correctly to forward / back / left / right (and yaw) stick commands
while running in **ACRO + Direct Flight Control mode** (`DF_MC_DIR_EN=1`).

In direct mode the sticks bypass all controllers and publish straight to the
control allocator (`mc_att_control_main.cpp:252`):

| Stick    | maps to              | scaled by      |
|----------|----------------------|----------------|
| roll     | torque X (roll)      | `DF_MC_DIR_RP` |
| pitch    | torque Y (pitch)     | `DF_MC_DIR_RP` |
| yaw      | torque Z (yaw)       | `DF_MC_DIR_YAW`|
| throttle | thrust Z (collective)| `DF_MC_DIR_THR`|

So "forward/back" = pitch torque, "left/right" = roll torque. The test drives
each axis at configurable amplitudes and records the actuator response so the
behaviour can be reviewed offline.

## Scope / non-goals

- Runs against **real, armed hardware secured on its wire**. Safety is the top priority.
- Does **not** try to fly or hold position — it commands raw torque/thrust in direct mode only.
- Does not modify vehicle parameters (only reads them for pre-flight checks).

## Connection

- MAVLink over **UDP**, default `udp:127.0.0.1:14550` (override with `--connect`).
- `pymavlink` (`mavutil`).

## Command injection

- **`MANUAL_CONTROL` message** streamed continuously at ~50 Hz from a background
  thread. This is exactly what direct mode consumes (`mavlink_receiver.cpp:2084`):
  - `x = pitch*1000`, `y = roll*1000`, `r = yaw*1000` (range ±1000)
  - `z = (throttle+1)*500` (range 0..1000; throttle −1..1)
- Continuous streaming keeps the input fresher than the FC's 100 ms direct-mode
  failsafe window, so the drone never trips the "no valid manual input" fallback
  mid-step.
- Prerequisite: `COM_RC_IN_MODE` must accept MAVLink manual control (1/2/3) and
  no physical TX should be overriding. The script reads and warns.

## Interactive console (raw terminal, single keypress)

Two states:

**IDLE** (disarmed, or armed but not testing):
| Key      | Action                                             |
|----------|----------------------------------------------------|
| `a`      | Arm (requires one-time typed `ARM` confirmation)   |
| `d`      | Disarm                                             |
| `p`      | Edit test parameters (only when **disarmed**)      |
| `1`–`9`,`0` | Run test 1–10 with current params (must be armed) |
| `q`      | Quit (auto-disarms first)                          |

**IN-TEST** (a sequence is executing):
| Key       | Action                                                        |
|-----------|---------------------------------------------------------------|
| **any key** | **Immediate emergency force-disarm** (`MAV_CMD_COMPONENT_ARM_DISARM`, param2=`21196`), abort sequence, return to IDLE |

Tests never auto-arm. Throttle holds `--base-throttle` during a test and returns
to idle between/after. Motors sit at idle throttle when armed but not testing.

## Test table (1–10)

Each "ramp" iterates the direction's amplitude list; each amplitude: center →
ramp → hold. Yaw centered unless noted.

| Key | Test                         | Direction(s)                    |
|-----|------------------------------|---------------------------------|
| 1   | Forward ramp                 | pitch +                         |
| 2   | Back ramp                    | pitch −                         |
| 3   | Right ramp                   | roll +                          |
| 4   | Left ramp                    | roll −                          |
| 5   | Full matrix                  | fwd, back, right, left          |
| 6   | Roll symmetry                | right ↔ left @ max amp          |
| 7   | Pitch symmetry               | fwd ↔ back @ max amp            |
| 8   | Yaw ramp                     | yaw +                           |
| 9   | Throttle-only baseline       | no torque, base throttle only   |
| 10  | Gentle all-directions        | fwd, back, right, left @ 1st amp|

## Parameters (overridable)

Defaults: amplitudes `30,40,50` (%) per direction, hold `3.0 s`, center `2.0 s`,
ramp `0.5 s`, base throttle `0.25`.

**Launch-time (CLI):**
```
--connect udp:127.0.0.1:14550
--amplitudes 30,40,50           # global default % list for all directions
--fwd-amp / --back-amp / --left-amp / --right-amp / --yaw-amp   # per-direction override
--hold 3.0  --center 2.0  --ramp 0.5
--base-throttle 0.25
--rate 50                       # MANUAL_CONTROL stream Hz
--dry-run                       # do everything except arm (validate harness)
--skip-checks                   # bypass DF_MC_DIR_EN / mode preconditions
```

**Interactive (`p`, disarmed only):** line-mode editor to retype any amplitude
list, hold, center, or base throttle without restarting. Current params are
printed before each test and recorded in the events log.

## Logging

Per run, a directory `logs/run_<timestamp>/` containing:
- `meta.json` — connection, params, firmware version/branch, key params (`DF_MC_DIR_*`, `CA_ROTOR*`, `COM_RC_IN_MODE`).
- `events.csv` — one row per command change: `t_wall, phase, test, direction, amplitude, pitch, roll, yaw, throttle`.
- Per-message telemetry CSVs (wide, flattened): `attitude.csv`, `attitude_target.csv`,
  `actuator_output_status.csv`, `servo_output_raw.csv`, `esc_status.csv`, `highres_imu.csv`.
- The **onboard `.ulg`** captures full-rate `actuator_motors`/`actuator_outputs`/
  `vehicle_torque_setpoint` in parallel; align by timestamp using `events.csv`.

## Safety

- Typed `ARM` confirmation before the first arm; explicit "props will spin, secure the drone" warning.
- Injector streams center + idle throttle **before** arming, so there is never a stale/zero-input window.
- `try/finally` + `SIGINT` + uncaught-exception → command center, **force-disarm**, restore terminal.
- Any key during a test → immediate force-disarm.
- `--dry-run` to validate the harness without arming.

## Success criteria ("acting as it should")

For each torque axis: the correct motor pair responds, the response reverses when
the stick reverses, motor deltas increase monotonically with 30→40→50%, and there
is no unexpected yaw coupling during pure pitch/roll commands.

## Files

- `df_direct_actuator_test.py` — the harness (single file).
- `README.md` — usage + safety.
- `requirements.txt` — `pymavlink`.
