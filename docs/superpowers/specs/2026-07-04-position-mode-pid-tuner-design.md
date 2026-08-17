# Position-Mode PID Tuner — Design Spec

Date: 2026-07-04
Target: SpinAir horizontal drone (MicoAir H743 v2), PX4 fork, branch `test_yaw_speed`
Author: Shahaf Shakuf

## Problem

The drone is a horizontal (x/y-only) tethered drone used for precise landing /
stabilization over a spot, in a GPS-denied area using optical flow. After moving
to a bigger airframe with more powerful fans, the drone "jumps" while trying to
hover over a spot — the camera jitters, making a stable fly-over impossible.

The operator is switching the flight configuration back to:
- **Direct Flight OFF** (`DF_MC_DIR_EN = 0`)
- **Position mode**
- **Yaw hold ON** (`DF_YAW_HOLD_EN = 1`)

...and needs to re-tune the PID gains for this bigger drone to get smooth
translation and steady hover.

### Prime suspect

`DF_ACC_PER_THR` (currently `0.5`) converts an acceleration setpoint to thrust:
`thrust_norm = acc_sp / DF_ACC_PER_THR`. Its own parameter documentation states
the bigger drone should be `~3.0`. If left too low, the controller commands ~6×
too much thrust for a given acceleration → actuator saturation → oscillation,
which matches the observed "jumping". This is the first lever to test, followed
by the horizontal velocity-loop gains.

## Goal

A single, standalone, interactive **Position-mode PID tuning console**
(`movementTest/df_position_tuner.py`) that:

1. Establishes the "smooth hover" baseline config (Direct off, Position mode,
   Yaw hold on).
2. Lets the operator edit 4 param groups live over MAVLink, with read-back
   verification.
3. Injects small, repeatable stick maneuvers to provoke a measurable response.
4. Reports objective smoothness metrics per trial + logs raw data, so gain sets
   can be compared and ranked.

## Scope / non-goals

- Runs against **real, armed hardware secured on its wire**. Safety is the top
  priority.
- Standalone new file in `movementTest/`. Does **not** refactor or modify the
  existing `df_direct_actuator_test.py` (proven, working). Some code duplication
  of common building blocks is accepted deliberately (per project conventions:
  minimal changes, no unrelated refactor).
- Excitation is **stick steps via `MANUAL_CONTROL`** only — no Offboard position
  setpoints in this version.
- Does not attempt automatic gain search / auto-tune; the operator drives the
  iteration. The tool provides config, excitation, and measurement.

## Architecture

Single self-contained Python file, `pymavlink`-based, following the structure and
conventions of `df_direct_actuator_test.py`:

- **MAVLink connection** — UDP default `udp:127.0.0.1:14550`, `--connect` override.
- **`MANUAL_CONTROL` injector** — background thread streaming at ~50 Hz
  (`x=pitch*1000, y=roll*1000, r=yaw*1000, z=(throttle+1)*500`), bounded to small
  amplitudes. Keeps input fresher than the FC failsafe window.
- **Telemetry logger** — background thread subscribing to and flattening the
  messages needed for metrics + offline analysis.
- **Raw single-key console** — cbreak terminal, IDLE / IN-TRIAL states.
- **Safety layer** — `try/finally` + SIGINT + uncaught-exception → force-disarm +
  terminal restore; any key aborts a trial with emergency force-disarm.
- **Param manager** — new capability: `PARAM_SET` with read-back verification,
  grouped editable param sets, export to a `.params` file.

### Reused vs. new

| Building block                     | Source                          |
|------------------------------------|---------------------------------|
| MAVLink connect / arm / disarm     | adapted from existing harness   |
| `MANUAL_CONTROL` injector          | adapted from existing harness   |
| Telemetry logger / flatten         | adapted from existing harness   |
| Raw-key console / HUD / safety     | adapted from existing harness   |
| **`PARAM_SET` + read-back**        | **new**                         |
| **Position-mode setup**            | **new**                         |
| **Excitation maneuvers**           | **new**                         |
| **Metric computation**             | **new**                         |

## Baseline config management

On startup (opt out with `--no-setup`), read and, if needed, set + verify by
read-back:

- `DF_MC_DIR_EN = 0` (Direct Flight off)
- `DF_YAW_HOLD_EN = 1`, `DF_YAWSPD_PID_EN = 1` (yaw hold active)
- Switch vehicle to **Position** flight mode via `MAV_CMD_DO_SET_MODE`.

Read-only sanity (warn, do not block — matching existing warn-only preflight):
`COM_RC_IN_MODE` (must accept MAVLink manual control), `DF_PAYLOAD_KG`, and
EKF / optical-flow health where available.

## Param tuning model

Editable param groups (edited via line-editor, **disarmed only**, matching the
existing `p` convention). Values apply live (RAM); `export` persists them.

- **Accel→thrust:** `DF_ACC_PER_THR`
- **Horizontal velocity PID:** `MPC_XY_P`, `MPC_XY_VEL_P_ACC`,
  `MPC_XY_VEL_I_ACC`, `MPC_XY_VEL_D_ACC`
- **Yaw hold:** `DF_YAWSPEED_P`, `DF_YAWSPEED_I`, `DF_YAWSPEED_D`,
  `DF_YAW_FINE_P`, `DF_YAW_FINE_I`, `DF_YAW_FINE_D`, `DF_YAWSPEED_MAXR`,
  `DF_YAW_ACC_MAX`
- **Payload scaling:** `DF_PAYLOAD_KG`, `DF_PAYLOAD_MIN`

Each write is verified by a follow-up read; a mismatch is surfaced as an error.
`export` writes current values to `logs/run_<ts>/tuned.params` (QGC-compatible
`name,value` format) and can optionally trigger an onboard param save so the set
survives reboot.

## Excitation maneuvers

All reuse the ~50 Hz injector, bounded to small amplitudes (configurable, small
defaults):

- **Hold / observe** (`h`) — center sticks, record steady-state jitter for N
  seconds. The most direct measure of "camera jumping while hovering."
- **Nudge & release** (`1`–`4`, fwd/back/left/right) — small step on one axis,
  then release to center; measures overshoot / oscillation / settling as the
  drone re-stabilizes.
- **Yaw step** (`y`, optional) — command a small heading change via `DF_YAW_HOLD`
  and measure yaw settling.

## Metrics & logging

Computed after each trial from telemetry (`LOCAL_POSITION_NED` velocity,
`ATTITUDE` roll/pitch/yaw + rates); printed as a one-line scorecard:

- Peak-to-peak **velocity oscillation** (vx / vy)
- Peak-to-peak **attitude wobble** (roll / pitch)
- **Overshoot %** (nudge test)
- **Dominant wobble frequency** — zero-crossing estimate (no new dependency;
  numpy FFT explicitly deferred)
- **Settling time** to a band
- Steady-state **RMS jitter** (hold test)

Per run, `logs/run_<ts>/` contains:
- `meta.json` — connection, params, firmware version/branch, key params.
- `events.csv` — one row per command change.
- `trials.csv` — one row per trial: trial type, active param values, and computed
  metrics, so gain sets can be ranked.
- Per-message telemetry CSVs (flattened) for offline plotting.
- `tuned.params` — on export.

## Interactive console

Raw single-key (cbreak), two states, matching the existing tool's UX.

**IDLE:**
| Key   | Action                                                    |
|-------|-----------------------------------------------------------|
| `a`   | Arm (one-time typed `ARM` confirmation)                   |
| `d`   | Disarm                                                    |
| `p`   | Edit a param group (disarmed only)                        |
| `h`   | Hold / observe trial (armed)                              |
| `1`–`4` | Nudge fwd / back / left / right (armed)                 |
| `y`   | Yaw-step trial (armed)                                    |
| `e`   | Export current tuned params to `.params`                  |
| `q`   | Quit (auto-disarms first)                                 |

**IN-TRIAL:** any key → **immediate emergency force-disarm** (`MAV_CMD_COMPONENT_ARM_DISARM`, param2=`21196`), abort trial, return to IDLE.

HUD line shows: flight mode, arm state, active gains, last scorecard.

## Safety

- Typed `ARM` confirmation before first arm; explicit "props will spin, secure
  the drone" warning.
- Injector streams center sticks before arming — no stale/zero-input window.
- `try/finally` + SIGINT + uncaught-exception → force-disarm + restore terminal.
- Bounded small stick amplitudes.
- Any key during a trial → immediate force-disarm.
- `--dry-run` to validate the harness without arming.

## Files

- `movementTest/df_position_tuner.py` — the tuner (single file).
- `movementTest/README_TUNER.md` — usage, safety, and a suggested tuning order
  (start with `DF_ACC_PER_THR`, then XY velocity P→D→I, then yaw hold).
- Reuses existing `requirements.txt` (`pymavlink`); no new dependency.

## CLI (planned)

```
--connect udp:127.0.0.1:14550   # MAVLink endpoint
--no-setup                      # skip auto config/mode setup (assume operator set it)
--rate 50                       # MANUAL_CONTROL stream Hz
--nudge-amp 0.15                # small stick step amplitude (fraction of full)
--hold-secs 5.0                 # hold/observe duration
--dry-run                       # do everything except arm
```

## Success criteria

- Establishes Direct-off / Position / Yaw-hold config and verifies it.
- Sets any of the 4 param groups live and confirms via read-back.
- Runs hold and nudge trials that produce a comparable scorecard.
- Operator can iterate gains and observe metrics trending toward lower
  oscillation, converging on a "no-jump" hover set, then export it.
