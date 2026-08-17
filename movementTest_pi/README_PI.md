# movementTest_pi — run the actuator test ON the Raspberry Pi

Goal: eliminate the WiFi hop (and possibly MAVProxy) that was throttling
`MANUAL_CONTROL` to ~5 Hz, so direct mode gets a steady >15 Hz input and we can
read the real actuator response.

Chain today: `PX4 —USB→ RasPi (MAVProxy) —WiFi→ laptop`.
Running the sender **on the Pi** removes the WiFi leg.

## Copy to the Pi

From the laptop:
```bash
scp -r movementTest_pi psdk@100.79.96.45:~/movementTest_pi
ssh psdk@100.79.96.45
cd ~/movementTest_pi
python3 -m pip install -r requirements.txt   # pymavlink
```

## ⚠️ SAFETY unchanged
Real armed drone, props spin, secure on the wire. Any key during a test =
force-disarm; Ctrl-C / exit force-disarm too. Start with `--dry-run`.

## Two ways to connect (try in this order)

### Option A — MAVProxy on localhost (try first, least disruptive)
Keeps MAVProxy/QGC/telemetry running. Removes only the WiFi hop.

Find a MAVProxy UDP output on the Pi (MAVProxy console: `output` — or check the
launch flags). It's usually something like `127.0.0.1:14550`. Then:
```bash
python3 df_direct_actuator_test.py --connect udp:127.0.0.1:14550 --dry-run
```
If MAVProxy has no localhost output, add one in the MAVProxy console:
```
output add 127.0.0.1:14550
```

**Verify it worked:** during a hold, the HUD `tx` should be ~40–50 and, more
importantly, grab the onboard `.ulg` afterward — `manual_control_setpoint` must
be **>15 Hz** (was 5 Hz). If it's still ~5 Hz → MAVProxy is coalescing
`MANUAL_CONTROL`; use Option B.

### Option B — direct USB serial (guaranteed clean, but stops MAVProxy)
MAVProxy holds the FC's USB port, so stop it first (frees `/dev/ttyACM0`):
```bash
# stop your MAVProxy (however it's started: systemctl / screen / tmux / kill)
python3 df_direct_actuator_test.py --connect /dev/ttyACM0 --baud 115200 --dry-run
```
(USB CDC ignores the actual baud value, but pymavlink needs one.)
Find the device with `ls /dev/ttyACM* /dev/ttyUSB*`.

## Run
```bash
# validate link + input rate first, no arming:
python3 df_direct_actuator_test.py --connect <endpoint> --dry-run
# watch HUD 'tx' and, after exit, tx_timing.csv summary

# real run (drone secured):
python3 df_direct_actuator_test.py --connect <endpoint>
#   a=arm  1=forward  d=disarm  q=quit   (any key during a test = disarm)
```

## What to check after a run
1. HUD during the hold: is the motor line `M[...]` **steady** now (not toggling)?
2. `logs/run_*/tx_timing.csv` summary line (printed on exit): median ~20 ms, 0 gaps >100 ms.
3. Onboard `.ulg`: `manual_control_setpoint` rate should be **>15 Hz**.

If input rate is fixed, a steady forward stick should give a steady actuator
response — then we can finally judge the forward-vs-yaw behavior on clean data.

## Notes
- CLI: `--connect`, `--baud`, `--rate` (uplink Hz, 50), `--tele-rate` (downlink Hz, 20),
  `--base-throttle`, `--hold`, `--center`, per-direction `--fwd-amp` etc., `--dry-run`, `--no-hud`.
- The HUD needs a TTY; over SSH that's fine. If you run detached, use `--no-hud`.
