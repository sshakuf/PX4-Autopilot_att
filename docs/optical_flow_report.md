# Optical Flow in PX4 — Complete Technical Report
## For Custom Target-Tracking Position Hold (No GPS)

---

## 1. Overview: What is Optical Flow

Optical flow sensors measure **angular motion of the scene** below the drone by tracking pixel movement frame-to-frame. The fundamental output is **integrated angle change in radians** — how much the scene appeared to rotate around each body axis during a time window.

The core insight: if you know the **height above ground** and the **angular flow rate** (rad/s), you can compute the **lateral velocity** of the drone:

```
velocity (m/s) = flow_rate (rad/s) × height_above_ground (m)
```

This is how PX4 derives position from optical flow — without GPS.

---

### Understanding Angular Motion: How Radians Become Physical Movement

**"What does angular actually mean? If the drone moves 1 meter in X, what does the sensor see?"**

#### The Core Geometry

Imagine your camera pointing straight down from height **h = 2 meters**:

```
         DRONE
          📷  ← camera looking down
          |
          | h = 2m
          |
   ───────┼──────── ground
          └── target
```

Now the drone moves **1 meter to the right**:

```
    BEFORE          AFTER
      📷     →→→    📷
      |              |
    2m|             2m|
      |              |
   ───A──────────────B───
      ↑              ↑
  target         target
```

The camera doesn't measure "I moved 1 meter". It measures the **angle** the target shifted in its field of view:

```
         📷 (new position, above B)
        /|
       / |
      /  | h = 2m
     /   |
    / θ  |
   A─────B
   |←1m→|

tan(θ) = opposite / adjacent = 1m / 2m = 0.5
θ = arctan(0.5) ≈ 0.46 radians ≈ 26.5°
```

Moving **1 meter at height 2m** produces **0.46 radians** of apparent angular shift.

#### The Formula

```
θ (radians) = physical_displacement (meters) / height (meters)

Rearranged to get back physical movement:
physical_displacement (m) = θ (rad) × height (m)
```

| Height | Drone moves 1m | Angular shift seen |
|--------|----------------|--------------------|
| 0.5 m  | 1 m            | 2.0 rad (huge)     |
| 1 m    | 1 m            | 1.0 rad            |
| 2 m    | 1 m            | 0.5 rad            |
| 5 m    | 1 m            | 0.2 rad            |
| 10 m   | 1 m            | 0.1 rad            |

> **Higher altitude = smaller angular shift for the same physical movement.** This is why height is critical — the same angular reading means totally different physical distances depending on altitude.

#### What the Sensor Actually Measures

The sensor measures **how fast the angle is changing** (angular rate in rad/s), then **integrates** (accumulates) it over a time window:

```
Frame 1 → Frame 2: target moved 50 pixels
Image is 640px wide, camera FOV is 1.2 radians horizontal

rad_per_pixel = 1.2 / 640 = 0.001875 rad/px
angular_shift = 50px × 0.001875 = 0.09375 radians  ← this is integrated_x in MAVLink

If this happened over dt = 0.014s (70 Hz):
angular_rate = 0.09375 / 0.014 = 6.7 rad/s  ← flow_rate used internally in EKF
```

#### Full Chain: Pixels → Radians → Velocity → Position Hold

```
1. Camera sees target shifted 50 pixels
        ↓
2. Convert to radians:
   integrated_x = 50px × (FOV / resolution) = 0.09375 rad
        ↓
3. EKF divides by dt to get angular rate:
   flow_rate = 0.09375 / 0.014s = 6.7 rad/s
        ↓
4. EKF subtracts gyro rotation (to isolate translation):
   flow_compensated = 6.7 - gyro_rate  (removes rotation from the signal)
        ↓
5. Multiply by height to get velocity:
   velocity = flow_compensated × height
   velocity = 6.7 rad/s × 2m = 13.4 m/s  ← "drone is moving at 13.4 m/s"
        ↓
6. Position controller sees non-zero velocity → applies braking
        ↓
7. Drone slows, target returns to center → flow goes to 0 → hover
```

#### Why Radians and Not Just Pixels?

Because **pixels are meaningless without knowing the camera FOV**. A 100-pixel shift on a narrow telephoto lens is a tiny angle. The same 100-pixel shift on a wide-angle lens is a huge angle. Converting to radians makes the measurement **camera-independent** — PX4 does not need to know your camera specs. You handle that conversion before sending the MAVLink message.

#### Quick Reference

| Concept | Meaning |
|---------|---------|
| `integrated_x = 0.09 rad` | Scene appeared to rotate 0.09 rad around X this time window |
| At height 2m → velocity 6.4 m/s | Drone was actually moving at 6.4 m/s horizontally |
| At height 5m → same reading → 16 m/s | Same sensor output = faster movement at higher altitude |
| flow = 0 | Scene not moving = drone stationary above target |
| flow = constant | Drone drifting at constant velocity |

**The key equation:**
```
velocity (m/s) = (integrated_rad / dt_seconds) × height_meters
```

---

## 2. MAVLink Message: `OPTICAL_FLOW_RAD`

This is **the message you need to publish** from your custom system. It is the primary interface for external optical flow sources.

**MAVLink message ID:** `106`
**Name:** `OPTICAL_FLOW_RAD`

### Fields

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `time_usec` | uint64 | µs | Timestamp |
| `sensor_id` | uint8 | - | Sensor ID (use 0) |
| `integration_time_us` | uint32 | µs | **Time window over which flow was accumulated** (e.g., 14285 µs = 70 Hz) |
| `integrated_x` | float | **radians** | **Flow around X axis (pitch axis, forward motion)** |
| `integrated_y` | float | **radians** | **Flow around Y axis (roll axis, lateral motion)** |
| `integrated_xgyro` | float | radians | Gyro rotation around X during same window |
| `integrated_ygyro` | float | radians | Gyro rotation around Y during same window |
| `integrated_zgyro` | float | radians | Gyro rotation around Z during same window |
| `temperature` | int16 | centi-°C | Sensor temperature (set 0 if unknown) |
| `quality` | uint8 | 0–255 | **Flow quality. 0=bad, 255=perfect. EKF ignores if too low** |
| `time_delta_distance_us` | uint32 | µs | Time window of distance measurement |
| `distance` | float | **meters** | **Height above ground (rangefinder). Set -1.0 if unknown** |

### Sign Convention

- `integrated_x` positive = scene rotates right-hand positive about X (nose-down → positive)
- `integrated_y` positive = scene rotates right-hand positive about Y (roll right → positive)
- These are **accumulated angles** over `integration_time_us`, not rates

---

## 3. Data Pipeline: From Sensor to Position Hold

```
┌─────────────────────────────────────────────────────────────────────┐
│                    YOUR CUSTOM SYSTEM                               │
│                                                                     │
│  Camera → Track target pixel movement → Compute dx_pixels, dy_pixels│
│         → Convert to radians → Publish OPTICAL_FLOW_RAD MAVLink msg │
└──────────────────────────────┬──────────────────────────────────────┘
                               │ OPTICAL_FLOW_RAD (MAVLink)
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│              MAVLINK RECEIVER  (mavlink_receiver.cpp:833)           │
│                                                                     │
│  Decodes OPTICAL_FLOW_RAD message                                   │
│  Maps:                                                              │
│    flow.integrated_x  → sensor_optical_flow.pixel_flow[0]          │
│    flow.integrated_y  → sensor_optical_flow.pixel_flow[1]          │
│    flow.integrated_*gyro → sensor_optical_flow.delta_angle[0..2]   │
│    flow.quality       → sensor_optical_flow.quality                │
│    flow.distance      → sensor_optical_flow.distance_m             │
│    flow.integration_time_us → sensor_optical_flow.integration_timespan_us│
│                                                                     │
│  Publishes → uORB: sensor_optical_flow                             │
└──────────────────────────────┬──────────────────────────────────────┘
                               │ uORB: sensor_optical_flow
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│              SENSORS MODULE  (sensors module)                       │
│                                                                     │
│  Applies SENS_FLOW_ROT (yaw rotation correction)                    │
│  Applies SENS_FLOW_SCALE (scale factor)                             │
│  Rate-limits to SENS_FLOW_RATE                                      │
│                                                                     │
│  Publishes → uORB: vehicle_optical_flow                             │
└──────────────────────────────┬──────────────────────────────────────┘
                               │ uORB: vehicle_optical_flow
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│              EKF2  (EKF2.cpp:UpdateFlowSample)                      │
│                                                                     │
│  Converts pixel_flow (radians) → flow_rate (rad/s):                 │
│    flow_rate = -pixel_flow / (integration_timespan_us * 1e-6)       │
│  Note: sign INVERTED here by EKF convention                         │
│                                                                     │
│  Creates flowSample:                                                │
│    .time_us    = midpoint of integration window                     │
│    .flow_rate  = [rad/s, rad/s]                                     │
│    .gyro_rate  = [rad/s, rad/s, rad/s]                              │
│    .quality    = 0–255                                              │
│                                                                     │
│  Pushes into flow_buffer (delayed to align with IMU timeline)       │
└──────────────────────────────┬──────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│        EKF2 FUSION  (optical_flow_control.cpp + fusion.cpp)         │
│                                                                     │
│  1. Gyro compensation:                                              │
│     flow_compensated = flow_rate - (gyro_rate - gyro_bias)         │
│     (removes rotational artifact → pure translational flow)         │
│                                                                     │
│  2. Range estimation:                                               │
│     range = height_above_ground / cos(tilt_angle)                  │
│                                                                     │
│  3. Velocity derivation:                                            │
│     vel_body_x = -flow_compensated_y * range                       │
│     vel_body_y =  flow_compensated_x * range                       │
│                                                                     │
│  4. EKF state update:                                               │
│     Fuses velocity measurement into horizontal position/velocity    │
│     state → constrains drift → position hold                        │
│                                                                     │
│  Publishes → uORB: vehicle_optical_flow_vel                         │
└──────────────────────────────┬──────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│         POSITION CONTROLLER uses estimated NED velocity/position    │
│         to maintain position hold                                   │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 4. uORB Messages Reference

### `sensor_optical_flow` — Raw input to EKF

```
pixel_flow[2]          (radians)    Accumulated optical flow angles
delta_angle[3]         (radians)    Accumulated gyro angles (same window)
delta_angle_available  (bool)       Whether gyro data is valid
distance_m             (meters)     Distance to ground center
distance_available     (bool)       Whether distance is valid
integration_timespan_us(µs)         Accumulation time window
quality                (0–255)      Quality indicator
max_flow_rate          (rad/s)      Sensor max measurable rate
min_ground_distance    (meters)     Min valid height
max_ground_distance    (meters)     Max valid height
```

### `vehicle_optical_flow_vel` — Output from EKF

```
vel_body[2]                 (m/s)    Velocity in body frame from optical flow
vel_ne[2]                   (m/s)    Velocity in NED frame
vel_body_filtered[2]        (m/s)    Low-pass filtered body velocity
vel_ne_filtered[2]          (m/s)    Low-pass filtered NED velocity
flow_rate_uncompensated[2]  (rad/s)  Raw flow rate
flow_rate_compensated[2]    (rad/s)  Gyro-compensated flow rate
gyro_rate[3]                (rad/s)  Synchronized gyro measurement
```

---

## 5. Critical: How Flow Becomes Velocity

The fundamental math chain (verified in source):

```
Step 1 - Gyro Compensation:
    flow_compensated = flow_rate - (gyro_rate - gyro_bias)
    // Removes rotation-induced apparent flow, leaves only translational

Step 2 - Range Estimation:
    range = hagl / R_to_earth(2,2)    // hagl / cos(tilt)
    // Converts angular rate to linear velocity

Step 3 - Body Velocity:
    vel_body_x = -flow_compensated_y * range
    vel_body_y =  flow_compensated_x * range
    // Note axis swap + sign convention

Step 4 - EKF Fusion:
    Predicted flow  = vel_body / range  (forward)
    Innovation      = predicted - measured
    State update    = Kalman gain × innovation
```

> **Key insight:** If you have no gyro data in your MAVLink message (send `NaN` for gyro fields), PX4 will use its own IMU gyro for compensation automatically (`EKF2_OF_GYR_SRC = Auto`).

---

## 6. EKF2 Parameters That Control Everything

### Primary Control Parameters

| Parameter | Default | Description | Your Setting |
|-----------|---------|-------------|--------------|
| `EKF2_OF_CTRL` | 1 | **Enable optical flow fusion. Must be 1** | 1 |
| `EKF2_OF_DELAY` | 20 ms | Measurement delay vs IMU. Match your pipeline latency | tune |
| `EKF2_OF_GATE` | 3.0 | Innovation gate (sigma). Higher = more tolerant of outliers | 3–5 |
| `EKF2_OF_N_MIN` | 0.15 rad/s | Noise at max quality (255) | tune |
| `EKF2_OF_N_MAX` | 0.5 rad/s | Noise at min quality | tune |
| `EKF2_OF_QMIN` | 1 | Min quality to fuse (in-air). **Set to 1 to accept almost anything** | 1 |
| `EKF2_OF_QMIN_GND` | 0 | Min quality on ground | 0 |
| `EKF2_OF_GYR_SRC` | 0 (Auto) | 0=use flow gyro if available, else IMU; 1=always IMU | 0 |
| `EKF2_OF_POS_X/Y/Z` | 0 | Camera offset from IMU in body frame (meters) | measure |

### Sensor Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SENS_FLOW_ROT` | 0 | Yaw rotation of sensor relative to body (0=0°, 1=45°, ...) |
| `SENS_FLOW_MINHGT` | 0.08 m | Minimum reliable height |
| `SENS_FLOW_MAXHGT` | 100 m | Maximum reliable height |
| `SENS_FLOW_MAXR` | 8 rad/s | Max angular rate measurable |
| `SENS_FLOW_RATE` | 70 Hz | Max publication rate |
| `SENS_FLOW_SCALE` | 1.0 | Scale factor applied to raw flow |

### Height Estimation (CRITICAL for no-GPS)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `EKF2_HGT_REF` | 1 | **Set to 2 (rangefinder) if you have one. Otherwise 0 (baro)** |
| `EKF2_RNG_CTRL` | 1 | Rangefinder fusion. Use if you have a rangefinder |
| `EKF2_TERR_MASK` | 3 | Terrain estimation sources (bit 0=rangefinder, bit 1=optical flow) |

---

## 7. Fusion Starting Conditions (Why Flow May Not Start)

The EKF logs this message when flow fails to start:
```
FLOW: Starting conditions FAIL - ctrl:X tilt:X within_dist:X qual:X mag:X tilt_ok:X counter:X terrain_valid:X horiz_aid:X timeout:X
```

All of these must be true simultaneously:

| Condition | Check | Notes |
|-----------|-------|-------|
| `EKF2_OF_CTRL == 1` | enabled | Must set parameter |
| `tilt_align` | IMU tilted correctly | True after ~2s flying |
| `hagl >= SENS_FLOW_MINHGT` | Not too close | Default: 0.08m |
| `hagl <= SENS_FLOW_MAXHGT` | Not too far | Default: 100m |
| `quality >= EKF2_OF_QMIN` | Good enough | In-air threshold |
| `flow_rate` not too large | Not saturated | < `SENS_FLOW_MAXR` |
| `flow_counter > 10` | Enough samples | ~0.15s at 70Hz |
| `terrain valid OR horiz aiding` | Has reference | **Critical for no-GPS** |
| `timeout > 2s since last fuse` | No rapid restart | Prevents oscillation |

> **For no-GPS operation:** The condition `terrain valid OR horiz_aid_active` is the hard one. Without GPS and without a rangefinder, EKF2 can't estimate terrain height. Solution options:
> 1. Use a rangefinder (best) — directly provides height above ground
> 2. Use barometer as height reference and trust initial ground height
> 3. The optical flow fusion will bootstrap itself once the terrain estimate converges

---

## 8. Position Hold Architecture (No GPS)

When optical flow is the **only** horizontal aiding source:

```
EKF State:
  - Horizontal velocity (vN, vE) ← constrained by optical flow
  - Horizontal position (pN, pE) ← integrated from velocity
  - Altitude ← from barometer or rangefinder
  - Gyro bias, accel bias ← estimated from redundancy

Dead Reckoning Mode:
  If inertial_dead_reckoning == true:
    "is_flow_required = true"
    → EKF MUST fuse flow or reset to flow velocity
    → Prevents unbounded drift

Position Hold (LOITER/POSCTL flight mode):
  → Uses estimated pN, pE, vN, vE
  → Optical flow keeps drift bounded
  → Without GPS: position error grows slowly from baro drift
     but velocity error is continuously corrected by flow
```

---

## 9. Building Your Custom Optical Flow Source

### What You Need to Send

Your system needs to publish `OPTICAL_FLOW_RAD` at a consistent rate (50–100 Hz recommended):

```python
# Pseudocode for your computer vision system

# Camera sees target at pixel (target_x, target_y) in frame 1
# In frame 2, target has moved to (target_x + dx_pixels, target_y + dy_pixels)
# Camera FOV: fov_x radians horizontal, fov_y radians vertical
# Image resolution: img_width x img_height pixels

# Pixel to radian conversion:
rad_per_pixel_x = fov_x / img_width   # e.g., 1.2 rad / 640 px = 0.001875 rad/px
rad_per_pixel_y = fov_y / img_height

# Flow in radians (accumulated over integration_time_us):
integrated_x = dx_pixels * rad_per_pixel_x   # positive = target moved right
integrated_y = dy_pixels * rad_per_pixel_y   # positive = target moved down

# Build MAVLink OPTICAL_FLOW_RAD message:
msg.time_usec             = current_time_us
msg.sensor_id             = 0
msg.integration_time_us   = time_since_last_measurement_us   # ~14285 at 70Hz
msg.integrated_x          = integrated_x                      # radians
msg.integrated_y          = integrated_y                      # radians
msg.integrated_xgyro      = float('nan')  # Let PX4 use IMU gyro
msg.integrated_ygyro      = float('nan')
msg.integrated_zgyro      = float('nan')
msg.temperature           = 0
msg.quality               = 200   # High quality from CV confidence
msg.time_delta_distance_us = integration_time_us
msg.distance              = height_m   # From rangefinder/estimate, or -1 if unknown
```

### Sign Convention for Your Camera (Looking Down)

Your camera is looking DOWN at a target. When the drone moves:

| Drone motion | Target on screen moves | `integrated_x` sign | `integrated_y` sign |
|---|---|---|---|
| Forward (nose direction) | Target moves UP in image | negative | negative |
| Backward | Target moves DOWN | positive | positive |
| Right | Target moves LEFT | negative | negative |
| Left | Target moves RIGHT | positive | positive |

> **Important:** Verify experimentally. The sign convention depends on your camera mounting orientation relative to the drone body axes. Use `SENS_FLOW_ROT` to correct yaw rotation, or handle it in your code.

### Position Hold Strategy: Target Locking

When your camera is tracking a target (e.g., landing pad):

```
Target stays at center of image → drone is NOT moving
Target drifts from center       → drone IS moving → compute flow → EKF corrects

Target offset from center (pixels):
  dx_pixels = target_x - image_center_x    (current error)

You DON'T directly command the drone to move.
Instead, you publish optical flow that represents the APPARENT MOTION.
EKF interprets this as velocity and the position controller nulls it out.

If target is at center: publish flow = 0 (or small noise)
If target drifted:      publish flow = offset * rad_per_pixel / dt
```

---

## 10. Height — The Critical Missing Piece

**Optical flow alone cannot determine position.** It measures angular rate. To get velocity you need:

```
velocity = angular_rate × height_above_ground
```

| Option | Accuracy | Recommendation |
|--------|----------|----------------|
| Dedicated rangefinder (TF-Luna, VL53L1X, etc.) | Best | **Strongly recommended** |
| Barometer (`EKF2_HGT_REF=0`) | ±1–5m | Use only for fixed-height operation |
| `distance` field in `OPTICAL_FLOW_RAD` | Same as rangefinder | Send height from any source here |

> You can send height in the `OPTICAL_FLOW_RAD` `distance` field directly — PX4 will use it as a terrain/distance estimate. This avoids needing a separate rangefinder topic.

---

## 11. Recommended Parameters for No-GPS Operation

```ini
# Enable optical flow as primary horizontal aiding
EKF2_OF_CTRL    = 1       # Enable optical flow fusion
EKF2_GPS_CTRL   = 0       # Disable GPS

# Height reference
EKF2_HGT_REF    = 2       # Rangefinder (if available), else 0 for baro

# Terrain estimation (needed for flow to start)
EKF2_TERR_MASK  = 3       # Enable from both rangefinder and flow

# Quality thresholds (permissive for CV-based system)
EKF2_OF_QMIN    = 1       # Accept quality >= 1
EKF2_OF_QMIN_GND = 0      # Accept any quality on ground

# Noise model
EKF2_OF_N_MIN   = 0.05    # Low noise for good CV tracking
EKF2_OF_N_MAX   = 0.3     # Worst case noise

# Innovation gate
EKF2_OF_GATE    = 3.0     # Increase to 5 if getting rejections

# Pipeline latency
EKF2_OF_DELAY   = 50      # Match your image processing delay (ms)

# Camera position relative to IMU
EKF2_OF_POS_X   = 0.0     # X offset (meters, body frame)
EKF2_OF_POS_Y   = 0.0     # Y offset
EKF2_OF_POS_Z   = 0.1     # Z offset (positive = below IMU)

# Valid height range
SENS_FLOW_MINHGT = 0.3    # Minimum operating height (meters)
SENS_FLOW_MAXHGT = 20.0   # Maximum operating height
```

---

## 12. Summary Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                    YOUR SYSTEM                                    │
│  Camera → CV Algorithm → Target dx/dy pixels                     │
│         → Convert: radians = pixels × (FOV / resolution)         │
│         → Send OPTICAL_FLOW_RAD via MAVLink @ 50-100 Hz         │
│           - integrated_x/y = radians (accumulated)               │
│           - quality = 200 (high, from your CV confidence)        │
│           - distance = height_m (from sensor or estimate)        │
│           - integration_time_us = dt since last message          │
└──────────────────────────────────┬───────────────────────────────┘
                                   │
                    OPTICAL_FLOW_RAD message
                                   │
                                   ▼
                    PX4 receives, maps to uORB
                    sensor_optical_flow →
                    vehicle_optical_flow
                                   │
                                   ▼
                    EKF2 converts radians → rad/s
                    Compensates gyro rotation
                    Multiplies by height → m/s velocity
                    Fuses into state estimate
                                   │
                                   ▼
                    Position controller uses
                    estimated position/velocity
                    to hold drone above target
```

---

## Key Takeaways

1. **Send `OPTICAL_FLOW_RAD`** — the exact MAVLink message PX4 consumes. No special driver needed.
2. **Fields that matter most:** `integrated_x`, `integrated_y` (radians), `integration_time_us`, `quality`, `distance`
3. **Send gyro as NaN** — PX4 will use its own IMU for gyro compensation automatically
4. **Quality field controls trust** — send 200–255 when CV is confident, lower when uncertain
5. **Height is essential** — provide it in `distance` field or via a rangefinder. Without height, EKF cannot compute velocity
6. **Publish at consistent rate** — 70 Hz ideal, minimum ~20 Hz. Inconsistent timing affects EKF timeline alignment
7. **Sign check experimentally** — move drone in known direction, verify `integrated_x/y` sign matches expectation before flying
8. **Set `EKF2_OF_DELAY`** to match your image processing pipeline latency (typically 30–100ms for CV systems)

---

*Report generated from PX4-Autopilot source analysis — branch: horizontal_att — 2026-03-21*
