# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Context: Tethered Horizontal Drone

**Vehicle Configuration:**
- **Horizontal-only movement** - Drone is tethered from above, preventing vertical movement
- **Horizontal rotor configuration** - Rotors are oriented horizontally for lateral thrust
- **2D stabilization focus** - Control system focuses on X/Y position and heading hold
- **Wire-constrained flight** - No altitude control needed, simplified dynamics

**Current Development Goal:**
Implementing stabilized flight mode with:
- **Heading hold** - Maintain desired yaw orientation
- **Position stabilization** - Counteract unwanted X/Y drift
- **Disturbance rejection** - Return to stable hover after external forces

## Build Commands

### Primary Build Targets
```bash
# SITL simulation (default target)
make px4_sitl_default

# Hardware targets (examples)
make px4_fmu-v5_default        # Pixhawk 4 (FMUv5)
make px4_fmu-v6x_default       # Pixhawk 6X
make ark_fmu-v6x_default       # ARK FMU-v6X
make holybro_durandal-v1_default

# Upload firmware to connected board
make px4_fmu-v5_default upload
```

### Development Commands
```bash
# Build and run all tests
make tests

# Run specific test
make px4_sitl_test test_results

# Format code (uses astyle)
make format

# Check code style
make check_format

# Static analysis
make clang-tidy

# Clean build
make clean

# Build with debugging info
make px4_sitl_default PX4_CMAKE_BUILD_TYPE=Debug
```

### Testing and Simulation
```bash
# Run SITL with Gazebo simulation
make px4_sitl_default gazebo

# Run integration tests with MAVSDK
make tests_integration

# Python unit tests and style checks
make python_coverage
```

## Code Architecture

### Core Flight Stack Structure

**Main Components:**
- **`src/modules/`** - Core flight modules (commanders, estimators, controllers)
- **`src/drivers/`** - Hardware drivers (sensors, actuators, communication)
- **`src/lib/`** - Reusable libraries and utilities
- **`platforms/`** - Platform-specific implementations (NuttX, POSIX)

### Key Flight Modules (for Tethered Horizontal Drone)

**Relevant Control Stack:**
- **`commander/`** - Flight mode manager, arming/disarming, failsafe handling
- **`flight_mode_manager/`** - High-level flight mode coordination
- **`mc_pos_control/`** - **PRIMARY MODULE** - X/Y position control and stabilization
- **`mc_att_control/`** - **CRITICAL** - Roll/pitch attitude control for horizontal positioning
- **`mc_rate_control/`** - **CORE** - Rate control for stabilization loops

**Less Relevant Modules:**
- **`fw_att_control/`** - Fixed-wing attitude control (not applicable)
- **`vtol_att_control/`** - VTOL attitude control (not applicable)

**Key Control Modifications Needed:**
- **Z-axis control** - Disable altitude/throttle control (tethered constraint)
- **Rotor mixing** - Modify for horizontal rotor configuration
- **Position controller** - Focus on X/Y stabilization without altitude coupling

**Estimation:**
- **`ekf2/`** - Extended Kalman Filter for state estimation
- **`sensors/`** - Sensor fusion and calibration
- **`attitude_estimator_q/`** - Quaternion-based attitude estimator

**Navigation:**
- **`navigator/`** - Mission execution, RTL, takeoff/landing
- **`landing_target_estimator/`** - Precision landing

### Hardware Abstraction

**Driver Architecture:**
- **Platform Layer** (`platforms/`) - OS abstraction (NuttX/POSIX)
- **Driver Layer** (`src/drivers/`) - Hardware-specific drivers
- **uORB** - Inter-process communication system

**Board Configuration:**
- **`boards/`** - Board-specific configurations (*.px4board files)
- Each board defines hardware mappings, default parameters, enabled modules

### Build System

**CMake Structure:**
- **Root CMakeLists.txt** - Main build configuration
- **Platform-specific cmake** (`platforms/*/cmake/`) - Toolchain and platform settings
- **Module CMakeLists.txt** - Individual module build definitions
- **Kconfig** - Configuration menu system

**Configuration System:**
- **px4board files** - Define target configurations
- **Kconfig** - Hierarchical configuration options
- **Parameters** - Runtime configuration system

### Message System (uORB)

**Message Definitions:**
- **`msg/`** - uORB message definitions (*.msg files)
- **Auto-generated** - C++ headers generated during build
- **Versioned messages** (`msg/versioned/`) - API-stable messages

**Key Message Types for Stabilization:**
- `vehicle_status` - Overall vehicle state and flight mode
- `vehicle_attitude` - **CRITICAL** - Roll/pitch angles for horizontal positioning
- `vehicle_local_position` - **CRITICAL** - X/Y position and velocities for stabilization
- `vehicle_angular_velocity` - Rate feedback for control loops
- `actuator_outputs` - **MODIFIED** - Horizontal rotor commands
- `sensor_combined` - Fused IMU data (gyro/accel)
- `manual_control_setpoint` - Pilot input commands
- `position_setpoint_triplet` - Target positions for stabilization

## Development Workflow

### Code Style
- Uses **astyle** for automatic formatting
- Follows Google C++ style guide conventions
- Run `make format` before committing
- Pre-commit hooks available in `Tools/astyle/`

### Testing Requirements
- Unit tests using **gtest** framework
- SITL integration tests with **MAVSDK**
- Hardware-in-the-loop (HIL) testing for hardware changes
- Flight log analysis using **Flight Review** (logs.px4.io)

### Parameter System
- Parameters defined in `*_params.c` files
- Auto-generated documentation and validation
- Runtime modification via MAVLink or QGroundControl
- Parameter groups for logical organization

### Logging and Debugging
- **ulog format** - Efficient binary logging
- **`logger`** module - Configurable topic logging  
- **MAVLink** - Real-time telemetry and commands
- **Console** - Debug shell via MAVLink or serial

## Important File Patterns

### Module Structure
```
src/modules/[module_name]/
├── CMakeLists.txt          # Build configuration
├── Kconfig                 # Configuration options
├── module.yaml            # Module metadata
├── [module]_main.cpp      # Module entry point
├── [module]_params.c      # Parameter definitions
└── [ModuleName].cpp/.hpp  # Main implementation
```

### Driver Structure
```
src/drivers/[category]/[driver_name]/
├── CMakeLists.txt         # Build configuration
├── [driver_name].cpp/.hpp # Driver implementation
└── module.yaml           # Driver metadata
```

### Platform Support
- **NuttX** - Real-time OS for flight controllers
- **POSIX** - Linux/macOS simulation and companion computers
- **QURT** - Qualcomm Hexagon DSP platform

### External Dependencies
- Managed via git submodules
- Located in respective module directories
- Updated with `make submodulesupdate`

## Tethered Drone Implementation Notes

### Control Loop Architecture for Stabilization

**Position Control Chain:**
1. **Position Controller** (`mc_pos_control/`) - Generates attitude setpoints from position errors
2. **Attitude Controller** (`mc_att_control/`) - Converts attitude setpoints to rate commands
3. **Rate Controller** (`mc_rate_control/`) - Generates actuator commands from rate errors

**Key Modifications for Horizontal Configuration:**

**1. Position Controller Changes:**
- Disable Z-axis position control (altitude locked by tether)
- Focus P/PID gains on X/Y position hold
- Implement heading hold (yaw position control)
- Relevant parameters: `MPC_XY_P`, `MPC_XY_VEL_P`, `MPC_XY_VEL_I`, `MPC_XY_VEL_D`

**2. Actuator Mixing Changes:**
- Modify `control_allocator/` for horizontal rotor configuration
- Update mixer files for lateral/longitudinal thrust mapping
- Consider creating custom vehicle type or modify existing multicopter mixing

**3. Sensor Configuration:**
- Ensure IMU orientation matches vehicle frame
- Verify magnetometer calibration for heading reference
- Consider GPS or optical flow for position feedback

**4. Flight Mode Development:**
- Create stabilized mode that maintains position and heading
- Implement manual control with position hold overlay
- Consider position setpoint smoothing for stability