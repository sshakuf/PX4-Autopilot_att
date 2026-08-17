# Position-Mode PID Tuner — `df_position_tuner.py`

Interactive MAVLink console to tune the SpinAir horizontal drone for **smooth
hover / translation** in **Position mode** with **Direct Flight OFF** and
**Yaw hold ON**. It sets params live, runs small bounded stick maneuvers, and
prints a smoothness scorecard so you can converge on gains that stop the camera
"jumping."

> ⚠️ **SAFETY**: this arms a real drone and spins the props. Secure the drone on
> its wire before arming. Any key during a trial = immediate force-disarm.
> Ctrl-C / crash / exit all force-disarm and restore the terminal.

## Install

```
pip install -r requirements.txt      # pymavlink only; no new deps
```

## Run

```
./df_position_tuner.py --connect udp:127.0.0.1:14550
```

On startup it (unless `--no-setup`):
- sets `DF_MC_DIR_EN=0`, `DF_YAW_HOLD_EN=1`, `DF_YAWSPD_PID_EN=1` (verified by read-back),
- switches the vehicle to **POSCTL**,
- reads and prints the current tunable params, and flags `DF_ACC_PER_THR` if it
  looks too low for the bigger drone.

## Keys

**IDLE**
| Key | Action |
|-----|--------|
| `a` | Arm (one-time typed `ARM` confirmation) |
| `d` | Disarm |
| `p` | Edit a param group (disarmed only) — sets live + read-back verify |
| `h` | Hold / observe trial (steady-state jitter — the "jumping" measure) |
| `1`/`2`/`3`/`4` | Nudge fwd / back / right / left, then measure the recovery |
| `y` | Single yaw step (steps `DF_YAW_HOLD`, measures yaw settling, then restores it) |
| `t` | **Auto-tune the yaw PID** for a stiff, strongly-resisting hold (see below) |
| `e` | Export current live params to `logs/run_<ts>/tuned.params` |
| `?` | Help |
| `q` | Quit (auto-disarms) |

**During a trial:** any key → **emergency force-disarm**.

## Param groups

| Group | Params |
|-------|--------|
| Accel→thrust | `DF_ACC_PER_THR` |
| Horizontal velocity PID | `MPC_XY_P`, `MPC_XY_VEL_P_ACC`, `MPC_XY_VEL_I_ACC`, `MPC_XY_VEL_D_ACC` |
| Yaw hold | `DF_YAWSPEED_P/I/D`, `DF_YAW_FINE_P/I/D`, `DF_YAWSPEED_MAXR`, `DF_YAW_ACC_MAX` |
| Payload scaling | `DF_PAYLOAD_KG`, `DF_PAYLOAD_MIN` |

Params set over MAVLink take effect live (RAM). Use `e` to write a `.params`
file; persist to the FC's permanent storage from QGC (or your usual flow) once
you're happy.

## Scorecards

- **HOLD**: `jitter vRMS x/y (m/s)` · `att p2p R/P (deg)` · `wobble R/P (Hz)`.
  Lower = smoother hover. This is the most direct "camera jumping" measure.
- **NUDGE**: `peak (deg)` · `overshoot %` · `settle (s)` · `osc (Hz)`.
  Lower overshoot / faster settle / no sustained oscillation = better.
- **YAW**: `yaw p2p (deg)` · `yaw RMS (deg)` after a heading step.

## Suggested tuning order

1. **`DF_ACC_PER_THR`** first. On the bigger fans this likely needs to go from
   `0.5` toward `~3.0`. Too low commands far too much thrust per unit
   acceleration → saturation → the jumping. Raise it until `h` (hold) jitter and
   nudge oscillation drop sharply. (See the param doc in
   `src/modules/mc_pos_control/multicopter_position_control_gain_params.c`.)
2. **`MPC_XY_VEL_P_ACC`** — reduce if still twitchy; increase if sluggish.
3. **`MPC_XY_VEL_D_ACC`** — add a little to damp overshoot/oscillation.
4. **`MPC_XY_VEL_I_ACC`** — last, to trim steady drift without inducing slow
   oscillation.
5. **Yaw hold** group — once translation is smooth, tune heading hold with `y`.
6. Confirm `DF_PAYLOAD_KG` matches the actual payload (it scales rate-loop gains).

## Automated yaw PID tuning (`t`)

Drives the yaw hold toward a **stiff, strongly-resisting** heading hold, fully
automatically. Arm first, then press `t`.

What it does each step:
1. Steps `DF_YAW_HOLD` between two headings 90° apart (alternating, so there's no
   net rotation drift on the wire).
2. Waits for the heading to settle (error < 3° and yaw-rate < 5°/s, held 0.7 s)
   or times out.
3. Scores the response: settling time, overshoot %, steady-state error, residual
   oscillation.
4. Tries raising one gain and keeps the change **only if the step stays stable
   AND the cost improves**, otherwise reverts.

It hill-climbs the six yaw gains, **P first** so stiffness goes up first:
`DF_YAWSPEED_P → DF_YAW_FINE_P → DF_YAWSPEED_D → DF_YAW_FINE_D → DF_YAWSPEED_I →
DF_YAW_FINE_I` (P/I ×1.3, D ×1.4), each clamped to its param bounds.
`cost = settling_s + 0.1·overshoot% + 0.5·ss_error°`.

A step is judged **unstable** (rejected + reverted) if it doesn't settle within
`--yt-settle-timeout`, overshoots past `--yt-overshoot-cap`, or leaves a sustained
wobble (>4° peak-to-peak in the last 1.5 s). The starting gains are treated as the
known-good floor — **it never leaves the drone on a worse-than-start set.**

When it finishes (converged, iteration cap, or you abort with any key), it applies
the **best stable** gains, restores your original `DF_YAW_HOLD`, and prints the
before/after. Press `e` to export. Per-iteration detail is logged to
`logs/run_<ts>/yaw_autotune.csv`.

> ⚠️ It rotates the drone ±90° repeatedly and writes gains live while armed. Keep
> a hand ready — any key aborts and force-disarms.

Flags: `--yt-max-iters 14`, `--yt-step-deg 90`, `--yt-overshoot-cap 25`,
`--yt-settle-timeout 12`. If P doesn't stiffen as much as you want, raise
`--yt-max-iters` (P is only retried once per 6-move pass).

## Options

```
--connect udp:127.0.0.1:14550   # MAVLink endpoint (or serial /dev/ttyACM0)
--no-setup                      # skip config/mode setup (you selected POSCTL)
--rate 50                       # MANUAL_CONTROL uplink Hz
--tele-rate 30                  # downlink telemetry Hz
--hover-throttle 0.0            # neutral throttle in POSCTL (-1..1); 0.0 = center/hold
--nudge-amp 0.15                # nudge stick step (fraction of full)
--push-secs 1.0                 # nudge hold before release
--settle-secs 4.0               # observation window after release / yaw step
--settle-band-deg 1.0           # attitude band that counts as "settled"
--hold-secs 5.0                 # hold/observe duration
--yaw-step-deg 15.0             # heading step for the single yaw trial ('y')
--yt-max-iters 14               # yaw auto-tune: max candidate steps
--yt-step-deg 90.0              # yaw auto-tune: alternating target separation
--yt-overshoot-cap 25.0         # yaw auto-tune: overshoot % -> 'unstable'
--yt-settle-timeout 12.0        # yaw auto-tune: max settle wait per step
--dry-run                       # do everything except arm
--no-hud                        # disable the live status line
```

## Logs

Per run, `logs/run_<ts>/`:
- `meta.json` — connection, options, params at start.
- `attitude.csv`, `local_position_ned.csv` — raw telemetry for offline plotting.
- `trials.csv` — one row per trial: trial type, direction, **all tunable param
  values**, and the computed metrics. Sort/compare rows to rank gain sets.
- `yaw_autotune.csv` — per-iteration log of the `t` auto-tune: move, accepted?,
  stability, metrics, and the full gain vector at each step.
- `tuned.params` — on export (`e`), QGC-compatible `name,value`.
