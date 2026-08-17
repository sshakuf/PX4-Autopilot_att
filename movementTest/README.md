# movementTest — Direct-Mode Actuator Movement Test

Interactive MAVLink harness to validate actuator response to
forward/back/left/right (+yaw) commands on the SpinAir horizontal drone in
**ACRO + Direct Flight Control mode** (`DF_MC_DIR_EN=1`).

See `SPEC.md` for the full design.

## ⚠️ SAFETY — read first

This **arms a real drone and spins the props.**

- Secure the drone on its wire/rope before arming.
- Keep clear of the props. Have a physical kill/battery cutoff ready.
- **Any key during a test = immediate force-disarm.**
- Ctrl-C, a crash, or normal exit all **force-disarm** and restore the terminal.
- Start with `--dry-run` to validate the harness without arming.

## Install

```bash
pip install -r requirements.txt
```

## Preconditions on the vehicle

- `DF_MC_DIR_EN = 1`  (direct mode on)
- `COM_RC_IN_MODE` = 1, 2, or 3  (accept MAVLink `MANUAL_CONTROL`)
- Vehicle in a manual mode (ACRO / MANUAL / STABILIZED — not position/altitude)
- A MAVLink UDP endpoint reachable (default `udp:127.0.0.1:14550`)

The script reads these params and refuses to run if they're wrong
(override with `--skip-checks`).

## Run

```bash
# validate the harness first — no arming
python3 df_direct_actuator_test.py --dry-run

# real run
python3 df_direct_actuator_test.py --connect udp:127.0.0.1:14550
```

### Keys

```
IDLE:   a = arm (types ARM confirmation once)
        d = disarm
        p = edit test parameters (only when disarmed)
        1-9,0 = run test 1..10 (must be armed)
        h = help    q = quit (auto-disarm)

IN-TEST: ANY key = EMERGENCY force-disarm
```

### Tests

| Key | Test |
|-----|------|
| 1 | Forward ramp (pitch +) |
| 2 | Back ramp (pitch −) |
| 3 | Right ramp (roll +) |
| 4 | Left ramp (roll −) |
| 5 | Full matrix (fwd/back/right/left) |
| 6 | Roll symmetry (right↔left @ max) |
| 7 | Pitch symmetry (fwd↔back @ max) |
| 8 | Yaw ramp (yaw +) |
| 9 | Throttle-only baseline (no torque) |
| 0 | Gentle all-directions @ min amp |

## Parameters

Defaults: amplitudes `30,40,50` %, hold `3.0 s`, center `2.0 s`, ramp `0.5 s`,
base throttle `0.25`.

Override at launch:

```bash
python3 df_direct_actuator_test.py \
    --amplitudes 30,40,50 \
    --fwd-amp 35,45 --yaw-amp 20,30 \
    --hold 3 --center 2 --ramp 0.5 \
    --base-throttle 0.25 --rate 50
```

Or interactively with `p` (disarmed only).

## Output

Each run writes `logs/run_<timestamp>/`:

- `meta.json` — connection, params, key vehicle params.
- `events.csv` — command timeline (`t_wall, phase, test, direction, amplitude, pitch, roll, yaw, throttle`).
- `attitude.csv`, `attitude_target.csv`, `actuator_output_status.csv`,
  `servo_output_raw.csv`, `esc_status.csv`, `highres_imu.csv` — streamed telemetry.

For full-rate actuator data, also download the onboard `.ulg` and align it with
`events.csv` by timestamp.
