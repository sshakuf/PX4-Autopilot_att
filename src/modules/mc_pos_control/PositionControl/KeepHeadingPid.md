# Keep Heading PID Flow in PositionControl

This document explains the keep-heading yaw controller in `PositionControl::update()`.
It focuses on the `DF_...` parameters and the two heading-control stages used to
produce the yaw-rate setpoint sent to the lower attitude/rate controller.

The controller in this file does not directly drive motors. It computes:

- `_yaw_sp`: the desired yaw angle
- `_yawspeed_sp`: the desired yaw-rate feed-forward setpoint

The lower PX4 attitude/rate controller then tries to make the vehicle follow
that yaw-rate setpoint using `MC_YAWRATE_*` parameters and actuator authority.

## Parameter Reference

### Enable and Target

`DF_YAW_HOLD_EN`

Enables the keep-heading override. When disabled, `PositionControl` uses the yaw
and yawspeed from the incoming trajectory setpoint. When enabled, it ignores the
trajectory yaw command and holds `DF_YAW_HOLD`.

`DF_YAW_HOLD`

The heading target in degrees. It is converted to radians and wrapped to
`[-pi, pi]`. In NED convention, `0 deg` is north, `90 deg` is east, `180 deg`
is south, and `270 deg` is west.

### Coarse Heading Controller

These parameters control the large-error heading acquisition stage.

`DF_YAWSPEED_MAXR`

Global maximum yaw-rate setpoint for keep-heading mode. All coarse and fine
commands are constrained to this value.

`DF_YAW_ACC_MAX`

Maximum coarse-mode yaw-rate acceleration. It limits how fast `_yawspeed_sp`
is allowed to rise when the controller is not in fine mode. It also affects the
coarse stopping profile:

```text
stopping_limited_rate = sqrt(2 * DF_YAW_ACC_MAX * abs_heading_error)
```

Lower values command earlier braking. Higher values let the controller stay
faster closer to the target, which can overshoot if the physical system cannot
brake that quickly.

`DF_YAWSPEED_P`

Coarse heading proportional gain. It converts absolute heading error to a yaw
rate:

```text
heading_rate = DF_YAWSPEED_P * abs_heading_error
```

The final coarse profile chooses the smallest of `DF_YAWSPEED_MAXR`,
`heading_rate`, and `stopping_limited_rate`.

`DF_YAWSPEED_I`

Coarse integral gain. In the current implementation, the shared integral state
is decayed in coarse mode instead of actively integrating:

```text
_yawspeed_integral *= 0.98
```

This prevents stale fine-mode trim from persisting during large turns.

`DF_YAWSPEED_D`

Coarse yaw-rate damping. It is used only when the vehicle is already rotating
toward the target faster than the coarse stopping profile allows:

```text
excess_yaw_rate = abs(measured_yaw_rate) - abs(profile_yaw_rate_sp)
yaw_rate_sp -= direction * DF_YAWSPEED_D * excess_yaw_rate
```

This is a braking term. Higher values command stronger opposite yaw-rate when
the payload is rotating too fast toward the target.

### Fine Heading Controller

These parameters control tight heading hold near the target. Fine mode is where
most sub-degree tuning happens.

`DF_YAW_FINE_ERR`

Heading-error threshold for entering fine mode. The controller enters fine mode
when either:

```text
abs_heading_error < DF_YAW_FINE_ERR
```

or the braking-compensated error says the vehicle is close enough after
accounting for current yaw rate:

```text
rotating_towards_target &&
abs(compensated_heading_error) < DF_YAW_FINE_ERR
```

`DF_YAW_FINE_RATE`

Fine-mode yaw-rate limit. This limits the proportional fine heading correction,
but the final yaw-rate setpoint is still globally constrained by
`DF_YAWSPEED_MAXR`.

`DF_YAW_FINE_P`

Fine proportional gain. It converts compensated heading error to a fine
yaw-rate command:

```text
fine_heading_rate = DF_YAW_FINE_P * compensated_heading_error
```

This is the primary tight-hold proportional action. Higher values react harder
to heading error, but too much creates oscillation because the command changes
sign aggressively around the target.

`DF_YAW_FINE_I`

Fine integral gain. It removes persistent heading offset caused by static bias,
asymmetric thrust, cable pull, payload torque, or any constant external yaw
force.

The integral stores heading error over time:

```text
_yawspeed_integral += heading_error * dt
yaw_rate_sp += DF_YAW_FINE_I * _yawspeed_integral
```

This is not a motor-output integral. It contributes yaw-rate setpoint.

`DF_YAW_FINE_D`

Fine damping gain on measured yaw rate. It subtracts measured yaw rate from the
fine heading command:

```text
yaw_rate_sp = fine_heading_rate - DF_YAW_FINE_D * measured_yaw_rate
```

Higher values resist rotation more strongly. Too low allows drift and overshoot.
Too high can fight the proportional command and cause sluggish or oscillatory
behavior.

`DF_YAW_FINE_ILIM`

Maximum fine integral contribution in `deg/s`. This limits how much yaw-rate
setpoint the fine integral is allowed to add. It protects against integral
windup.

`DF_YAW_FINE_ACC`

Fine-mode braking acceleration estimate and fine-mode slew limit.

It is used in two places:

1. To compute the stopping angle from current yaw rate:

```text
stopping_angle = yaw_rate * abs(yaw_rate) / (2 * DF_YAW_FINE_ACC)
compensated_heading_error = heading_error - stopping_angle
```

Lower values mean the controller assumes braking is weak, so it starts braking
earlier.

2. To slew-limit `_yawspeed_sp` in fine mode:

```text
max_delta_yaw_rate = DF_YAW_FINE_ACC * dt
_yawspeed_sp += constrain(yaw_rate_sp - previous_yawspeed_sp,
                          -max_delta_yaw_rate,
                           max_delta_yaw_rate)
```

Higher values let fine-mode correction power come in faster. Lower values make
the command smoother but slower.

`DF_YAW_FINE_TOL`

Fine heading tolerance. The current code uses it for helper logic, not as a hard
deadband that turns the controller off. It affects:

- the drift-arrest range
- the smooth deadband/static-friction boost scale

`DF_YAW_FINE_MINR`

Fine minimum-rate helper. It is not a hard final command in the current code.
It is used as:

- a cap for drift-arrest braking
- the maximum size of the smooth small-error boost

This helps overcome yaw deadband without snapping directly to a square-wave
minimum rate.

## Signal Flow

```text
vehicle state
  yaw, yaw_rate
       |
       v
target heading from DF_YAW_HOLD
       |
       v
heading_error = wrap_pi(target - yaw)
       |
       +--> coarse profile yaw-rate
       |
       +--> fine-mode decision
              |
              v
         fine PID / trim / damping
              |
              v
      integral and braking helpers
              |
              v
      constrain to DF_YAWSPEED_MAXR
              |
              v
      slew limit to _yawspeed_sp
              |
              v
vehicle_attitude_setpoint.yaw_sp_move_rate
              |
              v
lower attitude/rate controller and actuators
```

## `update()` Step by Step

### 1. Validate Input and Run Position Control

`update(dt)` first checks `_inputValid()`. If valid, it runs normal position and
velocity control:

```cpp
_positionControl();
_velocityControl(dt);
```

Yaw hold is layered on top of that. It changes the yaw fields of the output
attitude setpoint, not the horizontal/vertical position control calculation.

### 2. Enable Keep Heading Override

When `_keep_heading_enabled` is true:

```cpp
_yaw_sp = _keep_heading_target;
```

This forces the desired yaw angle to the DF target.

The code also protects timing and parameter values:

```text
dt_limited = constrain(dt, 0.002, 0.04)
max_yaw_rate = max(DF_YAWSPEED_MAXR, 1 deg/s)
max_yaw_accel = max(DF_YAW_ACC_MAX, 1 deg/s^2)
```

The constrained `dt` avoids very large or very small integration and slew-limit
steps.

### 3. Compute Heading Error and Direction

```text
heading_error = wrap_pi(target_yaw - current_yaw)
abs_heading_error = abs(heading_error)
direction = +1 when heading_error >= 0, otherwise -1
rotating_towards_target = measured_yaw_rate * direction > 0
```

`wrap_pi()` is important. It chooses the shortest yaw direction across the
`-180/180 deg` boundary.

Example:

```text
target = 0 deg
current = 350 deg
heading_error = +10 deg, not -350 deg
```

### 4. Estimate Stopping Angle

Fine mode needs to know not only where the barrel is, but also how much it will
continue to rotate if it starts braking now.

```text
stopping_angle = yaw_rate * abs(yaw_rate) / (2 * DF_YAW_FINE_ACC)
compensated_heading_error = heading_error - stopping_angle
```

This is the rotational equivalent of:

```text
distance_to_stop = velocity^2 / (2 * acceleration)
```

The sign is preserved by `yaw_rate * abs(yaw_rate)`. If yaw rate is positive,
the stopping angle is positive. If yaw rate is negative, the stopping angle is
negative.

`compensated_heading_error` is then used by the fine proportional controller.
This lets the controller brake before the raw heading error crosses zero.

### 5. Decide Coarse Mode vs Fine Mode

Fine mode is active when:

```text
abs_heading_error < DF_YAW_FINE_ERR
```

or:

```text
rotating_towards_target &&
abs(compensated_heading_error) < DF_YAW_FINE_ERR
```

The second condition allows early transition to fine mode when the vehicle is
still physically far away but moving fast enough that the stopping prediction is
near the target.

## Coarse Heading Controller

Coarse mode is for large heading errors. It is not a full motor PID. It is a
heading-to-yaw-rate profile with damping and slew limiting.

### Coarse P Term

```text
heading_rate = DF_YAWSPEED_P * abs_heading_error
```

This means large errors request larger yaw rates. Small errors request smaller
yaw rates.

### Coarse Braking Profile

```text
stopping_limited_rate = sqrt(2 * DF_YAW_ACC_MAX * abs_heading_error)
```

This gives the maximum yaw rate that can theoretically stop inside the remaining
heading error, assuming the vehicle can decelerate at `DF_YAW_ACC_MAX`.

### Coarse Profile Output

```text
profile_yaw_rate_sp =
    direction * min(DF_YAWSPEED_MAXR,
                    heading_rate,
                    stopping_limited_rate)
```

The profile chooses the most conservative limit:

- `DF_YAWSPEED_MAXR`: never command more than this
- `heading_rate`: proportional to error
- `stopping_limited_rate`: slow down near target

At this point:

```text
yaw_rate_sp = profile_yaw_rate_sp
```

### Coarse D Braking

After integral handling, the coarse controller checks if actual yaw rate is
already too high:

```text
excess_yaw_rate = abs(measured_yaw_rate) - abs(profile_yaw_rate_sp)
```

If not in fine mode, rotating toward the target, and excess is positive:

```text
yaw_rate_sp -= direction * DF_YAWSPEED_D * excess_yaw_rate
```

This commands braking only when needed. It does not damp normal motion toward
the target.

### Coarse Slew Limit

In coarse mode, when the command is accelerating toward the target, the command
is slew-limited by `DF_YAW_ACC_MAX`:

```text
max_delta = DF_YAW_ACC_MAX * dt
_yawspeed_sp = previous + constrain(yaw_rate_sp - previous,
                                    -max_delta,
                                     max_delta)
```

When braking or correcting an overspeed condition, the command can change
immediately because the controller should be allowed to stop the motion.

## Fine Heading PID

Fine mode is the tight heading-hold controller. It uses:

- P on braking-compensated heading error
- I on raw heading error
- D on measured yaw rate
- drift-arrest and boost helpers for static bias/deadband
- fine slew limiting through `DF_YAW_FINE_ACC`

The structure is:

```text
fine P:   DF_YAW_FINE_P * compensated_heading_error
fine D: - DF_YAW_FINE_D * measured_yaw_rate
fine I: + DF_YAW_FINE_I * integral(heading_error)
helpers: drift arrest and smooth boost
limit:   DF_YAW_FINE_RATE and DF_YAWSPEED_MAXR
slew:    DF_YAW_FINE_ACC
```

### Fine P: Heading Pull to Target

```text
fine_heading_rate =
    constrain(DF_YAW_FINE_P * compensated_heading_error,
              -DF_YAW_FINE_RATE,
               DF_YAW_FINE_RATE)
```

This is the main fine heading correction. It uses
`compensated_heading_error`, not raw `heading_error`, so it can brake before
crossing the target.

### Fine D: Measured Yaw-Rate Damping

```text
yaw_rate_sp = fine_heading_rate - DF_YAW_FINE_D * measured_yaw_rate
```

This damps motion. If the barrel is rotating left, this adds a command to oppose
that rotation. If the barrel is rotating right, it commands left braking.

The D term reacts immediately to motion. It does not wait for heading error to
build.

### Drift-Arrest Helper

The current code has an extra helper for the exact case where the barrel is near
the target but an external force starts rotating it away.

```text
drift_arrest_error = max(3 * DF_YAW_FINE_TOL, 2 deg)
drifting_away_from_target = measured_yaw_rate * heading_error < 0
```

The sign test means:

- heading error says the target is one way
- measured yaw rate is moving the vehicle the other way

If the vehicle is within `drift_arrest_error`, drifting away, and yaw rate is
above `0.5 deg/s`, the controller computes:

```text
drift_arrest_rate = constrain(-2 * DF_YAW_FINE_D * measured_yaw_rate,
                              -DF_YAW_FINE_MINR,
                               DF_YAW_FINE_MINR)
```

If this drift-arrest command is stronger than the existing command, it replaces
the command:

```text
if abs(drift_arrest_rate) > abs(yaw_rate_sp):
    yaw_rate_sp = drift_arrest_rate
```

Important behavior:

- It reacts to measured yaw-rate drift before heading error gets large.
- `DF_YAW_FINE_MINR` is only a cap here.
- It is continuous, not an on/off square command.

### Smooth Static-Friction Boost

When not using drift arrest, the code applies a smooth boost if the vehicle is
nearly stopped and outside a small error threshold:

```text
if abs_heading_error > 0.25 deg
and abs(measured_yaw_rate) < 5 deg/s:
    boost_scale = constrain(abs_heading_error / DF_YAW_FINE_TOL, 0, 1)
    yaw_rate_sp += direction * DF_YAW_FINE_MINR * boost_scale
```

This helps overcome static yaw deadband. It ramps in gradually:

- at `0 deg` error: no boost
- at half tolerance: half boost
- at tolerance or above: full `DF_YAW_FINE_MINR` boost

The result is constrained to `DF_YAW_FINE_RATE`.

### Fine I: Trim Against Constant Bias

The integral gain is chosen from the active mode:

```text
yaw_i_gain = fine_heading_hold ? DF_YAW_FINE_I : DF_YAWSPEED_I
```

In fine mode, the integral limit is:

```text
integral_yaw_rate_limit = DF_YAW_FINE_ILIM
integral_limit = DF_YAW_FINE_ILIM / DF_YAW_FINE_I
```

The code computes the current integral contribution:

```text
yaw_rate_i = DF_YAW_FINE_I * _yawspeed_integral
yaw_rate_unsaturated = yaw_rate_sp + yaw_rate_i
saturated = abs(yaw_rate_unsaturated) >= DF_YAWSPEED_MAXR
```

Then it integrates only when it is safe:

```text
if fine mode and (not saturated or integral would reduce saturation):
    _yawspeed_integral += heading_error * dt
```

The anti-windup condition is:

```text
!saturated || (heading_error * yaw_rate_unsaturated < 0)
```

Meaning:

- if not saturated, integrate normally
- if saturated, only integrate when the heading error would push the command
  back toward zero instead of deeper into saturation

Finally:

```text
yaw_rate_sp += DF_YAW_FINE_I * _yawspeed_integral
```

This is the slow trim term that should hold against a constant external torque
without requiring a permanent heading error.

### Fine Slew Limit

After all P/I/D/helper terms, the command is constrained:

```text
yaw_rate_sp = constrain(yaw_rate_sp, -DF_YAWSPEED_MAXR, DF_YAWSPEED_MAXR)
```

Then fine mode applies a yaw-rate setpoint slew limit:

```text
max_delta_yaw_rate = DF_YAW_FINE_ACC * dt
_yawspeed_sp = previous_yawspeed_sp +
               constrain(yaw_rate_sp - previous_yawspeed_sp,
                         -max_delta_yaw_rate,
                          max_delta_yaw_rate)
```

This prevents abrupt square-wave rate setpoints in fine mode.

## The Two Heading Control Stages

### Stage 1: Coarse Heading-to-Rate Profile

Purpose: get from large heading error toward the target without commanding more
rate than the system can stop.

Inputs:

- raw heading error
- measured yaw rate
- `DF_YAWSPEED_P`
- `DF_YAWSPEED_D`
- `DF_YAWSPEED_MAXR`
- `DF_YAW_ACC_MAX`

Output:

- coarse yaw-rate setpoint

Main behavior:

```text
large error -> higher yaw-rate request
near target -> stopping profile lowers yaw-rate request
too much measured yaw-rate -> D braking subtracts from command
```

### Stage 2: Fine Heading PID

Purpose: hold tightly near the target and reject external torque.

Inputs:

- compensated heading error
- raw heading error for integral
- measured yaw rate
- `DF_YAW_FINE_P`
- `DF_YAW_FINE_I`
- `DF_YAW_FINE_D`
- `DF_YAW_FINE_RATE`
- `DF_YAW_FINE_ACC`
- `DF_YAW_FINE_TOL`
- `DF_YAW_FINE_MINR`
- `DF_YAW_FINE_ILIM`

Output:

- fine yaw-rate setpoint

Main behavior:

```text
P: pull toward the target using predicted stopping error
D: oppose measured yaw-rate immediately
I: build trim against constant external torque
drift arrest: react before small drift grows
boost: overcome static deadband smoothly
slew limit: prevent setpoint jumps
```

## Tuning Interpretation

If the yaw-rate setpoint is smooth and near the value you expect, but measured
yaw rate does not follow, the problem is not solved by increasing
`DF_YAW_FINE_P`. The lower yaw-rate loop or actuator yaw authority is then the
bottleneck.

Use the plots this way:

```text
Yaw angle error large, yaw-rate setpoint small:
    increase heading/fine gains or rate limits.

Yaw-rate setpoint jumps in square waves:
    reduce DF_YAW_FINE_MINR, reduce DF_YAW_FINE_P, or lower helper aggressiveness.

Yaw-rate setpoint is reasonable but measured yaw rate is delayed or too small:
    tune MC_YAWRATE_P/I/FF/K or check actuator/mixer yaw authority.

Yaw overshoots target at high speed:
    lower DF_YAW_FINE_ACC or DF_YAW_ACC_MAX so braking starts earlier,
    or raise damping if the measured yaw-rate loop can track.

Persistent offset with low yaw rate:
    increase DF_YAW_FINE_I or DF_YAW_FINE_ILIM carefully.
```

## Units

The parameters are exposed in degrees and degrees per second, but the code
stores and calculates yaw internally in radians:

```text
degrees -> math::radians(...) at parameter load
rad/s and rad/s^2 inside PositionControl
published yaw_sp_move_rate is rad/s
plots often convert it back to deg/s
```

Always check whether a plot is showing degrees or radians before comparing
values directly to code variables.
