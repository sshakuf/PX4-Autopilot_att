# Agent context — IR beacon target-hold debugging

Session handoff. Written 2026-08-15. Branch `ir_cam`, board `micoair_h743-v2`.

## The project

PX4 fork for a **horizontal-only** drone hanging on a rope: it moves in X/Y only,
no vertical control. GPS-denied, so horizontal state comes from **optical flow**.
Goal is precise landing/stabilisation above an **IR beacon** seen by a
downward-facing LSDT spot camera on a UART.

Build: `make micoair_h743-v2_default` (add `upload` to flash).

## Where we are

**Original symptom:** commanding target-hold produced a *runaway* — the offset
grew instead of shrinking.

**Cause found and fixed:** `DF_IRC_ROT` was `0`, should be `2` (camera mounted
180°). Confirmed by bench test in all four directions and by re-verifying the
whole math chain numerically against a log. **This is closed.**

**But the retest still did not converge**, and not because of direction — the
drone barely moved at all. That is the current open question, and the leading
suspect is the optical flow, not the IR camera.

**Immediate next action:** analyse `TestScripts/testA.csv` (5,479 rows, recorded
2026-08-14 18:13, never analysed) and/or run Test A below.

## Status board

| # | Item | Confidence | State |
|---|---|---|---|
| 1 | IR sign (`DF_IRC_ROT` 0 → 2) | **confirmed** | ✅ fixed + verified |
| 8 | IR lateral axis inverted (image mirrored in x) | **confirmed symptom** | 🔧 patched 2026-08-17, awaiting flight test |
| 9 | Yaw hold: 3.2 Hz divergence, then too weak | **confirmed** | 🔧 params changed, still wanders ±30° |
| 2 | Optical flow cannot resolve our velocities | **downgraded** | 🔽 flow does produce signal at h≈1 m |
| 7 | Rotation-induced phantom velocity in the flow | **suspected** | 🔍 open, new leading hypothesis |
| 3 | Drone does not move when commanded | **confirmed fact** | 🔴 open, cause unknown |
| 4 | IR spot selection fragile | confirmed in data | ⚠️ latent risk |
| 5 | FOV geometry / `DF_TGT_MAX_R=3.0` unreachable | confirmed | ⚠️ needs param change |
| 6 | `DF_TGT_I` not zeroed for tests | confirmed | 📋 procedural |

### 1. IR sign — CLOSED

Bench test, beacon in four positions, camera down, drone stationary:

| beacon | dx_px | dy_px |
|---|---|---|
| front | +15 / −68 | **+226 / +287** |
| back | +40 / +79 | **−423 / −433** |
| right | **−319 / −385** | +58 / −160 |
| left | **+320 / +346** | −155 / −217 |

Mapping is `X_fwd → +down_img`, `Y_right → −right_img`; matrix `[[0,−1],[1,0]]`,
determinant **+1** (a proper rotation, no mirror). Only `ROT 2` satisfies all
four. Verified live afterwards: front `angle_x=+0.269`, right `angle_y=+0.369`.

### 2. Optical flow resolution — the leading suspect

Every `pixel_flow` value in the log is an integer multiple of **0.00213 rad**
over a fixed 10 ms integration window:

```
0.00213 rad / 0.01 s      = 0.213 rad/s per LSB
× height                  = 0.213 · h  m/s per LSB
at h = 0.46 m             = 0.098 m/s per LSB
```

Actual motion in the test was 0.02 m/s = **0.2 LSB** — unmeasurable. 8 of 16
logged samples read exactly zero. You cannot close a velocity loop at
0.02–0.25 m/s with 0.1 m/s feedback resolution.

Caveat: `sensor_optical_flow` is logged at exactly 1.00 Hz, which is the
*logger's* sample interval — the true publish rate is unknown from that log.
`pixel_flow[1]` was zero in all 16 sensor samples but nonzero in
`vehicle_optical_flow`, so **do not** claim the y axis is dead; 16 samples is too
few.

**Downgraded 2026-08-15** by `TestScripts/testA.csv`: at h≈0.96 m with real
handling the flow reports velocities up to 0.86 m/s and segment means of
0.03–0.44 m/s. It is not stuck at zero. The original worry was extrapolated from
16 decimated samples at h=0.46 m. Resolution is still coarse (~0.2 m/s per LSB at
1 m) but this is no longer the leading hypothesis.

### 7. Rotation-induced phantom velocity — NEW leading hypothesis

From `testA.csv` (106 s, 20 Hz, no gaps):

```
integrated path length (∫|v|dt) = 11.65 m
net displacement                =  0.14 m
max excursion from start        =  0.72 m
total yaw rotation              =  1626 deg  (4.5 turns)
```

The EKF reports 11.65 m of travel that goes nowhere while the vehicle spins.
Consistent with flow not being fully compensated for rotation.

Concrete lead: **`EKF2_OF_POS_X/Y/Z` are all 0**, i.e. the EKF assumes the flow
sensor sits exactly on the rotation axis. If it is physically offset, yaw creates
genuine sensor translation that gets attributed to vehicle motion. Measure the
offset and set those params.

Also unexplained: the EKF's own `∫v·dt = [+0.062, +0.421]` does not match its
`Δpos = [−0.065, −0.130]` over the same window, with no CSV gaps. Do not
overclaim — EKF position is not a naive integral of the published velocity.

### 3. Drone does not move — the actual blocker

From `log_2_2026-8-13-16-21-40.ulg`, target-hold active 13.4–28.5 s:

| | commanded | actual |
|---|---|---|
| mean speed | 0.202 m/s | **0.020 m/s** |
| travel over 9.3 s | ~1.9 m | 0.10 m N, 0.12 m E |

Body thrust reached only 0.21 normalised on 2 of 4 motors (`control[1]`,
`control[2]` ≈ 0.24; `control[0]`, `control[3]` ≈ 0) — the controller is not even
straining. Three candidate causes, not yet separated: the rope physically holding
the drone, flow blindness (#2), or thrust authority.

Because it never moved, the offset grew in all three segments
(0.205→0.386, 0.158→0.265, 0.264→0.385 m). **That growth is not evidence of a
sign error.**

### 4. IR spot health

- `valid=False` in 25/222 samples (11%)
- dropouts of **0.92 s** (t≈17.8) and **1.44 s** (t≈28.5); the first reset the
  tracker mid-run — visible as `integral` zeroed at 18.4, restarting at 19.0
- `spot_score` ∈ {0,4,9,16,28,37,45,90,127}; 50/222 below 32
- `spot_id` switches between 0 and 2

`IrCam.cpp:184-190` takes `spots[0]` unconditionally with **no score threshold**,
and `count` (payload[0]) is read but never published. If the camera reports
multiple spots, slot 0 can switch physical targets between frames → the offset
teleports → velocity slams. Not yet proven to happen in flight.

### 5. FOV geometry

Half-angles from the intrinsics: lateral `atan(618/1047)=30.53°`, fore/aft
`atan(480/1065)=24.26°`. Footprint = `0.590·h` lateral, `0.451·h` fore/aft.

| height | lateral | fore/aft | flow LSB |
|---|---|---|---|
| 0.5 m | ±0.30 m | ±0.23 m | 0.11 m/s |
| 0.7 m | ±0.41 m | ±0.32 m | 0.15 m/s |
| 1.2 m | ±0.71 m | ±0.54 m | 0.26 m/s |
| 1.5 m | ±0.89 m | ±0.68 m | 0.32 m/s |

**The two sensors want opposite heights.** IR footprint grows with h; flow
resolution degrades with h. Use low (~0.5 m) for flow tests, ~1.2 m for
target-hold work.

Consequences:
- `DF_TGT_MAX_R = 3.0` m is **unreachable at any flyable height** — the capture
  radius in `TargetHold.cpp:107` can never trigger. Should be ~half the footprint
  (~0.4 m at 1.2 m).
- The original "beacon 0.5 m off-centre" test was physically impossible at
  h≈0.46 m (limit ±0.21 m fore/aft). The log confirms it: peak `angle_y=0.577`
  → 0.30 m, pinned at the frame edge (`dx_px≈−600`, half-width 618).

### 8. IR lateral axis inverted — patched, not yet flight-verified

From `log_1_2026-8-17-11-13-22.ulg` (target hold active 30 s, IR valid 597/607):

```
|offset_fwd |   0.178 -> 0.080 -> 0.126 m   converging/flat
|offset_right|  0.064 -> 0.290 -> 0.276 m   DIVERGING (4.5x)
thrust points toward beacon: forward 100%, lateral 89%   <- software is consistent
```

Forward closes, lateral runs away, while the *commanded* thrust points at the
beacon on both axes. So the net lateral loop is inverted somewhere downstream of
the command.

Forward-correct + lateral-inverted needs `(angle_x, angle_y) = (+down, +right)`,
which **none of the four `DF_IRC_ROT` rotations can produce** — all four are
proper rotations and this needs a reflection. Ruled out by analysis:
motor-permutation (`(0 2)(1 3)` inverts Fy *and* yaw, but yaw works), yaw
estimate error (180° inverts both axes, 90° swaps them), and `CA_ROTOR*`
geometry (verified self-consistent with the allocator's Fx/Fy/τz bases).

**Patch applied** in `IrCam.cpp` `handleFrame()`: mirror the column at the parse
site, `x_corrected = FRAME_WIDTH-1-x`, keeping `DF_IRC_ROT = 2`. Done at the
source rather than by negating `_dx_px` so the `raw_x` reconstruction in
`publishReport()` stays a real pixel coordinate for `DF_IRC_CX` and `k1`.
Verified numerically: `angle_y` flips sign for every off-centre pixel, `angle_x`
perturbed by ≤0.0085° (k1 `r2` coupling), `|angle_y|` drifts ≤0.057° (1 px
mirror offset) — both far under the several-px spot jitter.

**IMPORTANT caveat.** Flipping *any single link* in the chain flips the net
result, so this patch will fix the observed behaviour whether or not the camera
is the actual culprit. It is validated by the flight symptom, **not** by an
independent bench reading. The 30-second confirming test was never run: place the
beacon toward the nose (expect `dy_px > 0`, known good), then to the drone's
right at the same radius and read `dx_px`. Positive ⇒ camera really is mirrored
and the patch sits in the right place. Negative ⇒ the camera was fine, the true
fault is downstream, and the logged `dx_px`/`angle_y` are now sign-wrong even
though flight behaves correctly. Worth doing before trusting IR logs again.

Note also `DF_IRC_CX` should strictly become `617` (mirrored principal point);
1 px was left alone but must be re-derived if CX is ever calibrated off-centre.

### 9. Yaw hold

Two failure modes found and addressed, in `log_18`/`log_23`/`log_5` of 2026-08-15:

1. **3.2 Hz diverging oscillation** — `rate` grew 44→339 °/s while `rate_sp`
   stayed a calm +23 °/s, torque saturated (±1.59, motors railed). The *outer*
   loop was innocent. Causes: `MC_YAW_TQ_CUTOFF = 5 Hz` adding ~33° phase lag
   right at the oscillation frequency, `MC_YAWRATE_D = 0`, and I-windup.
   Suspected physical driver: rope torsional resonance.
2. **Then far too weak** — after cutting gain, armed drift (−6.1 °/s) matched
   free-hanging drift (−4.9 °/s): the controller had no effect. Compounded by
   `DSHOT_MIN = 0` leaving `control[0]` pinned at 0.000 with baseline thrust only
   0.131, so `unallocated_torque[2]` reached 0.072 of a commanded 0.103 — ~70% of
   the yaw torque never reached the motors. Yaw torque is a *differential about a
   baseline*, and thrust ∝ ω², so near-idle there is almost no authority.

Current: `MC_YAW_TQ_CUTOFF 0`, `MC_YAWRATE_K 0.35`, `MC_YAWRATE_I 0.08`,
`MC_YAWRATE_D 0.002`, `MC_YAWRATE_FF 0`, `DSHOT_MIN 0.001`, `DF_YAWSPEED_P 0.5`,
`DF_YAW_FINE_EN 1`. Still **wanders ±30°** (target −70°, actual −93°..−31°) —
needs another pass. Yaw wander also rotates the NED→body thrust mapping, so it
adds lateral tracking error.

`DSHOT_MIN` is safe to raise: on this symmetric airframe a *uniform* thrust
component is wrench-neutral, so it buys differential headroom without net force.

**Which enable applies depends on mode** — `DF_YAW_HOLD_EN` in POSCTL/ALTCTL
(`MulticopterPositionControl.cpp:321`), `DF_ATT_HOLD_EN` only in
manual/stabilized (`mc_att_control_main.cpp:493`). Two separate
`HeadingHoldControl` instances share the same `DF_YAW*` params. Also: heading
hold is nested inside `if (_inputValid())` in `PositionControl::update`, so if
the x/y setpoint chain goes invalid, **yaw hold stops too**.

Target hold and heading hold are otherwise **orthogonal** and run together
fine — target hold writes velocity[0..1], heading hold writes yaw/yawspeed.

## Verified-correct math chain (do not re-litigate)

Checked numerically against `log_2_2026-8-13-16-21-40.ulg`:

| stage | check | result |
|---|---|---|
| dx/dy → angles | Brown-Conrady by hand | matches to 5 dp |
| angles → `offset_NE` | `q.rotateVector` + `h/los_down` | matches |
| offset → `vel_cmd` | `0.5·offset + I` | exact |
| `vel_cmd` → controller | `vehicle_local_position_setpoint.vx/vy` | **identical** |
| vel error → acc | `MPC_XY_VEL_P_ACC(3.0)·(sp.vx − vx)` | −0.609 vs −0.602 |
| acc_NED → thrust_body | `R(yaw)ᵀ·acc / DF_ACC_PER_THR(4.0)` | matches to 3 dp, 12 points |

`_q.rotateVector(los_body)` is body→NED and is **correct as written**.

## Traps that already cost time

1. **`trajectory_setpoint` in a ulog is the flight-task INPUT**, not the
   post-override setpoint. It reads velocity 0 even when target-hold is driving.
   The real controller input is **`vehicle_local_position_setpoint`**.
2. **`SDLOG_MODE = 0` means "when armed until disarm"** (PX4 default) — it does
   *not* log from boot. To log disarmed: `logger on` / `logger off` in nsh
   (overrides arming, no reboot), or `SDLOG_MODE=2` + reboot.
3. **MAVSDK *can* read debug messages** via the `mavlink_direct` plugin
   (`drone.mavlink_direct.message("DEBUG_FLOAT_ARRAY")`). It has no `telemetry`
   API for them, which is easy to mistake for "impossible".
4. **`DEBUG_VECT` cannot carry two different records.**
   `MavlinkStreamDebugVect` does one `_debug_sub.update()` per tick and
   `debug_vect` has queue depth 1, so back-to-back publications silently drop all
   but the last. Use one `debug_array`.
5. **pymavlink defaults to a MAVLink 1 dialect**, where `DEBUG_FLOAT_ARRAY`
   (id 350) does not exist. Set `MAVLINK20=1` and `MAVLINK_DIALECT=common`
   *before* importing pymavlink.
6. **A constant yaw offset cannot fake a mirror** (rotation, det +1), but a yaw
   that *changes between* readings can. Take direction readings back-to-back with
   the nose direction verified each time.
7. **A claimed sign error was walked back.** An apparent EKF-vs-IR displacement
   sign disagreement (EKF `[−0.094,−0.121]` vs IR-implied `[+0.103,+0.207]`) is
   most likely drift below the flow resolution floor, not an inverted axis. Check
   resolution before alleging sign bugs.

## Code changes in the working tree (uncommitted)

### `src/drivers/ir_cam/IrCam.hpp`, `IrCam.cpp` — TEMPORARY

`#define IR_CAM_DEBUG_MAVLINK 1` block. Mirrors every `ir_camera_report` into
`debug_array`, which PX4 streams as `DEBUG_FLOAT_ARRAY` named `IRCAM` at 50 Hz
over USB (`MAVLINK_MODE_CONFIG`, `mavlink_main.cpp:1723-1725`). Needed because
`ir_camera_report` is a custom uORB topic no MAVLink client can subscribe to.

`data[]`: `0 dx_px, 1 dy_px, 2 angle_x, 3 angle_y, 4 valid, 5 spot_id,
6 spot_score`. NaN angles pass through unchanged.

**Set the define to 0, or delete the two fenced blocks, to remove.** Builds
clean at 88.4% flash; `DEBUG_FLOAT_ARRAY` and `IRCAM` both confirmed present in
the ELF.

### `TestScripts/` — new

- `ir_flow_monitor.py` — live body-frame view: beacon position, camera footprint,
  drifting dots for flow, velocity arrow, HUD, CSV recording, arm/disarm button.
  `--selftest` (parser + DEBUG_FLOAT_ARRAY unpack) and `--demo` (no hardware).
  CSV is 23 columns and **includes `roll_deg`/`pitch_deg` and the three body
  rates** — added 2026-08-15 because without tilt the beacon offset cannot be
  compensated after the fact, and `yawspeed` is needed to test #7. Files recorded
  before that date have the old 18-column schema and cannot be tilt-corrected.
- `mavsdk_read_ircam.py` — minimal MAVSDK reader, proves the `mavlink_direct`
  path works. Cannot run alongside the monitor (mavsdk_server claims the port).
- `README.md`, `requirements.txt`

Arm button: label follows the **real** state from `HEARTBEAT.base_mode &
MAV_MODE_FLAG_SAFETY_ARMED`, never the requested state. Arm needs two clicks
within 3 s, disarm needs one. `COMMAND_ACK` decoded on screen. The arm round-trip
has **only been tested against the demo stub** — never against a real vehicle.

Two logical commits when wanted: firmware diagnostic, and test tooling.

## Key parameter values (from the 16:21:40 log)

```
DF_IRC_ROT   2      (fixed this session)
DF_IRC_FX    1047   DF_IRC_FY 1065   DF_IRC_CX 618   DF_IRC_CY 480   DF_IRC_K1 -0.2845
DF_TGT_P     0.5    DF_TGT_I  0.05   DF_TGT_D  0.0   DF_TGT_ILIM 0.3
DF_TGT_VMAX  0.5    DF_TGT_MAX_R 3.0 (unreachable)   DF_TGT_HOLD_EN 1
DF_ACC_PER_THR 4.0  MPC_XY_VEL_P_ACC 3.0
SDLOG_MODE   0      SENS_FLOW_ROT 0   EKF2_OF_CTRL 1
```

`DF_TGT_I` was **not** set to 0 for the last test despite the plan; the
integrator wound to −0.095 on both axes and became 78% of the P term, which makes
that log useless as P-tuning data.

## Next steps, in order

1. ~~Analyse `TestScripts/testA.csv`~~ — **done 2026-08-15, it is NOT Test A.**
   The prescribed maneuver was not performed: max excursion 0.72 m (not 2–3 m),
   1626° of yaw (nose was supposed to be fixed), h≈0.96 m (not 0.5 m).
   Proof independent of the EKF: the beacon stayed in view 82% of 106 s with a
   max offset of 0.75 m against an FOV limit of ±0.57 m lateral — a 2–3 m walk
   would have lost it permanently. Findings folded into #2 and #7 above.
   An attempted IR-vs-EKF position cross-check gave weak, sign-inconsistent
   correlations (+0.27 N, +0.20 E) but is **not trustworthy**: that CSV lacked
   roll/pitch, and at h≈1 m a 10° tilt is 0.17 m of phantom offset, as large as
   the whole signal. Schema has since been fixed — **re-record, do not re-analyse
   the old file.**
2. **Test A — is the flow usable?** Disarmed, drone *held in hands*, low (~0.5 m),
   level, nose fixed. Hold 5 s → walk **2–3 m straight at ~0.5 m/s** → hold 5 s →
   walk back → hold 5 s. Compare `disp body` against a tape measure.
   - tracks within ~20%, right sign → flow healthy, #3 was the rope
   - near zero while clearly walking → flow blind, fix before any tuning
   - right magnitude, wrong sign → inverted axis, check `SENS_FLOW_ROT`
   - Use ≥1.5 m of travel: drift floor is ~0.1 m, so a 0.4 m move is ambiguous.
3. Only if flow is healthy: `DF_TGT_I 0`, fix `DF_TGT_MAX_R`, fly at ~1.2 m, and
   capture P-tuning data.
4. Backlog: spot-selection robustness (#4) — publish `count`, add a score
   threshold, pick best spot instead of `spots[0]`.

## Analysis environment

- Logs: `~/Documents/QGroundControl Daily/Logs/` — latest analysed
  `log_2_2026-8-13-16-21-40.ulg` (16 s, target-hold active 13.4–28.5 s).
- `pyulog` installed; `ulog_info` at `~/.local/bin/ulog_info`.
- Read topics with `pyulog.ULog(path)`; params via `u.initial_parameters`.
- Relevant topics: `ir_camera_report`, `target_hold_status`,
  `vehicle_local_position`, `vehicle_local_position_setpoint`,
  `vehicle_thrust_setpoint`, `vehicle_attitude`, `sensor_optical_flow`,
  `estimator_aid_src_optical_flow`, `actuator_motors`.

## Source map

| File | Role |
|---|---|
| `src/drivers/ir_cam/IrCam.cpp` | UART/MSP parse, pixel→angle, ROT mapping (`:264-285`), spot selection (`:184-190`) |
| `src/drivers/ir_cam/ir_cam_params.c` | `DF_IRC_*` |
| `src/modules/mc_pos_control/TargetHold/TargetHold.cpp` | offset (`:40-71`), PID (`:138-162`) |
| `src/modules/mc_pos_control/multicopter_target_hold_params.c` | `DF_TGT_*` |
| `src/modules/mc_pos_control/MulticopterPositionControl.cpp` | target-hold override (`:627-653`), thrust NED→body (`:842-874`) |
| `msg/IrCameraReport.msg`, `msg/TargetHoldStatus.msg` | message definitions |
