/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

#include "MulticopterRateControl.hpp"

#include <circuit_breaker/circuit_breaker.h>
#include <drivers/drv_hrt.h>
#include <mathlib/math/Functions.hpp>
#include <mathlib/math/Limits.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

// NOTE: This file intentionally contains only rate-control logic.
// Any attitude/heading shaping or manual-stick interpretation
// should happen upstream (e.g., attitude controller).

// Static member definition
MulticopterRateControl *MulticopterRateControl::_myobject = nullptr;

MulticopterRateControl::MulticopterRateControl(bool vtol)
    : ModuleParams(nullptr),
      WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
      _vehicle_thrust_setpoint_pub(
          vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc)
               : ORB_ID(vehicle_thrust_setpoint)),
      _vehicle_torque_setpoint_pub(
          vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc)
               : ORB_ID(vehicle_torque_setpoint)),
      _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")) {
  _vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

  parameters_updated();
  _controller_status_pub.advertise();
}

MulticopterRateControl::~MulticopterRateControl() { perf_free(_loop_perf); }

bool MulticopterRateControl::init() {
  if (!_vehicle_angular_velocity_sub.registerCallback()) {
    PX4_ERR("callback registration failed");
    return false;
  }
  return true;
}

void MulticopterRateControl::parameters_updated() {
  // Rate control parameters
  // The controller gain K converts parallel form (P + I/s + sD)
  // to the ideal form K * [1 + 1/(s Ti) + s Td]
  const Vector3f rate_k{_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(),
                        _param_mc_yawrate_k.get()};

  _rate_control.setPidGains(rate_k.emult(Vector3f(_param_mc_rollrate_p.get(),
                                                  _param_mc_pitchrate_p.get(),
                                                  _param_mc_yawrate_p.get())),
                            rate_k.emult(Vector3f(_param_mc_rollrate_i.get(),
                                                  _param_mc_pitchrate_i.get(),
                                                  _param_mc_yawrate_i.get())),
                            rate_k.emult(Vector3f(_param_mc_rollrate_d.get(),
                                                  _param_mc_pitchrate_d.get(),
                                                  _param_mc_yawrate_d.get())));

  _rate_control.setIntegratorLimit(Vector3f(_param_mc_rr_int_lim.get(),
                                            _param_mc_pr_int_lim.get(),
                                            _param_mc_yr_int_lim.get()));

  _rate_control.setFeedForwardGain(Vector3f(_param_mc_rollrate_ff.get(),
                                            _param_mc_pitchrate_ff.get(),
                                            _param_mc_yawrate_ff.get()));

  // manual rate control acro mode rate limits (kept for safety / fallbacks)
  _acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()),
                            radians(_param_mc_acro_p_max.get()),
                            radians(_param_mc_acro_y_max.get()));

  _output_lpf_yaw.setCutoffFreq(_param_mc_yaw_tq_cutoff.get());
}

int debugcounter = 0;

void MulticopterRateControl::Run() {
  if (should_exit()) {
    _vehicle_angular_velocity_sub.unregisterCallback();
    exit_and_cleanup();
    return;
  }

  perf_begin(_loop_perf);

  // Parameters update
  if (_parameter_update_sub.updated()) {
    parameter_update_s param_update;
    _parameter_update_sub.copy(&param_update);

    updateParams();
    parameters_updated();
  }

  /* run controller on gyro changes */
  vehicle_angular_velocity_s angular_velocity;

  if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

    const hrt_abstime now = angular_velocity.timestamp_sample;

    // Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
    const float dt =
        math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
    _last_run = now;

    const Vector3f rates{angular_velocity.xyz};
    const Vector3f angular_accel{angular_velocity.xyz_derivative};

    /* check for updates in other topics */
    _vehicle_control_mode_sub.update(&_vehicle_control_mode);

    if (_vehicle_land_detected_sub.updated()) {
      vehicle_land_detected_s vehicle_land_detected;

      if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
        _landed = vehicle_land_detected.landed;
        _maybe_landed = vehicle_land_detected.maybe_landed;
      }
    }

    _vehicle_status_sub.update(&_vehicle_status);

    // Default rate setpoint is the current rates (safe fallback)
    _rates_setpoint = rates;

    // Default thrust setpoint zeroed
    _thrust_setpoint.zero();

    // Use rates setpoint topic if available
    vehicle_rates_setpoint_s vehicle_rates_setpoint{};

    if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint) &&
        _vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) {

      _rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)
                               ? vehicle_rates_setpoint.roll
                               : rates(0);
      _rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch)
                               ? vehicle_rates_setpoint.pitch
                               : rates(1);
      _rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)
                               ? vehicle_rates_setpoint.yaw
                               : rates(2);

      _thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
    }

    // Run the rate controller
    if (_vehicle_control_mode.flag_control_rates_enabled) {

      // reset integrator if disarmed
      if (!_vehicle_control_mode.flag_armed ||
          _vehicle_status.vehicle_type !=
              vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
        _rate_control.resetIntegral();
      }

      // update saturation status from control allocation feedback
      control_allocator_status_s control_allocator_status;

      if (_control_allocator_status_sub.update(&control_allocator_status)) {
        Vector<bool, 3> saturation_positive{};
        Vector<bool, 3> saturation_negative{};

        if (!control_allocator_status.torque_setpoint_achieved) {
          for (size_t i = 0; i < 3; i++) {
            if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
              saturation_positive(i) = true;

            } else if (control_allocator_status.unallocated_torque[i] <
                       -FLT_EPSILON) {
              saturation_negative(i) = true;
            }
          }
        }

        // TODO: send the unallocated value directly for better anti-windup
        _rate_control.setSaturationStatus(saturation_positive,
                                          saturation_negative);
      }

      // run rate controller
      Vector3f torque_setpoint = _rate_control.update(
          rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);

      // apply low-pass filtering on yaw axis to reduce high frequency torque
      // caused by rotor acceleration
      torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt);

      // Apply torque overrides BEFORE conversion (if enabled)
      if (_torque_override_enabled) {
        torque_setpoint(0) = _torque_override(0);
        torque_setpoint(1) = _torque_override(1);
        torque_setpoint(2) = _torque_override(2);
      }

      // publish rate controller status
      rate_ctrl_status_s rate_ctrl_status{};
      _rate_control.getRateControlStatus(rate_ctrl_status);
      rate_ctrl_status.timestamp = hrt_absolute_time();
      _controller_status_pub.publish(rate_ctrl_status);

      // publish thrust and torque setpoints
      vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
      vehicle_torque_setpoint_s vehicle_torque_setpoint{};

      // HORIZONTAL DRONE MODIFICATION:
      // For horizontal drone configuration:
      // - Roll and pitch torques from attitude controller are converted to X/Y
      // thrust
      // - Torque is only used for yaw (clockwise/counter-clockwise rotation)
      // - The conversion gain determines how much torque translates to thrust

      const float horiz_gain = _param_mc_horiz_t2tq.get();

      if (horiz_gain > 0.0f) {
        // Horizontal drone configuration
        // Convert roll/pitch torques to X/Y thrust for horizontal movement
        // Pitch torque (forward/backward tilt) -> X thrust (forward/backward
        // movement) Roll torque (left/right tilt) -> Y thrust (left/right
        // movement)

        // Pitch torque controls forward/backward movement (X axis)
        // Positive pitch torque = nose down = forward movement
        vehicle_thrust_setpoint.xyz[0] = PX4_ISFINITE(torque_setpoint(1))
                                             ? torque_setpoint(1) * horiz_gain
                                             : 0.f;

        // Roll torque controls left/right movement (Y axis)
        // Positive roll torque = roll right = move right
        // Negative sign to match coordinate system convention
        vehicle_thrust_setpoint.xyz[1] = PX4_ISFINITE(torque_setpoint(0))
                                             ? -torque_setpoint(0) * horiz_gain
                                             : 0.f;

        // No vertical thrust - drone is tethered
        vehicle_thrust_setpoint.xyz[2] = 0.0f;

        // Only use yaw torque for rotation (clockwise/counter-clockwise)
        // Zero out roll and pitch torques as they've been converted to thrust
        vehicle_torque_setpoint.xyz[0] =
            0.0f; // Roll torque converted to Y thrust
        vehicle_torque_setpoint.xyz[1] =
            0.0f; // Pitch torque converted to X thrust
        vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(torque_setpoint(2))
                                             ? torque_setpoint(2)
                                             : 0.f; // Yaw torque for rotation
      } else {
        // Standard vertical drone configuration
        _thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);

        // Use all three torque axes for standard configuration
        vehicle_torque_setpoint.xyz[0] =
            PX4_ISFINITE(torque_setpoint(0)) ? torque_setpoint(0) : 0.f;
        vehicle_torque_setpoint.xyz[1] =
            PX4_ISFINITE(torque_setpoint(1)) ? torque_setpoint(1) : 0.f;
        vehicle_torque_setpoint.xyz[2] =
            PX4_ISFINITE(torque_setpoint(2)) ? torque_setpoint(2) : 0.f;
      }

      // scale setpoints by battery status if enabled
      if (_param_mc_bat_scale_en.get()) {
        if (_battery_status_sub.updated()) {
          battery_status_s battery_status;

          if (_battery_status_sub.copy(&battery_status) &&
              battery_status.connected && battery_status.scale > 0.f) {
            _battery_status_scale = battery_status.scale;
          }
        }

        if (_battery_status_scale > 0.f) {
          for (int i = 0; i < 3; i++) {
            vehicle_thrust_setpoint.xyz[i] = math::constrain(
                vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f,
                1.f);
            vehicle_torque_setpoint.xyz[i] = math::constrain(
                vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f,
                1.f);
          }
        }
      }

      // Apply thrust overrides after conversion (if enabled)
      // This ensures manual thrust commands work correctly for both
      // configurations
      if (_thrust_override_enabled) {
        vehicle_thrust_setpoint.xyz[0] = _thrust_override(0);
        vehicle_thrust_setpoint.xyz[1] = _thrust_override(1);
        vehicle_thrust_setpoint.xyz[2] = _thrust_override(2);
      }

      // Debug output before publishing
      if (_debug_enabled) {
        mavlink_log_info(&_mavlink_log_pub,
                         "[DEBUG] %d  Thrust: X=%.3f Y=%.3f Z=%.3f | Torque: "
                         "R=%.3f P=%.3f Y=%.3f",
                         debugcounter++, (double)vehicle_thrust_setpoint.xyz[0],
                         (double)vehicle_thrust_setpoint.xyz[1],
                         (double)vehicle_thrust_setpoint.xyz[2],
                         (double)vehicle_torque_setpoint.xyz[0],
                         (double)vehicle_torque_setpoint.xyz[1],
                         (double)vehicle_torque_setpoint.xyz[2]);
      }

      vehicle_thrust_setpoint.timestamp_sample =
          angular_velocity.timestamp_sample;
      vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
      _vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

      vehicle_torque_setpoint.timestamp_sample =
          angular_velocity.timestamp_sample;
      vehicle_torque_setpoint.timestamp = hrt_absolute_time();
      _vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

      // Store latest setpoints for debugging
      _last_thrust_setpoint = vehicle_thrust_setpoint;
      _last_torque_setpoint = vehicle_torque_setpoint;

      updateActuatorControlsStatus(vehicle_torque_setpoint, dt);
    }
  }

  perf_end(_loop_perf);
}

void MulticopterRateControl::updateActuatorControlsStatus(
    const vehicle_torque_setpoint_s &vehicle_torque_setpoint, float dt) {
  for (int i = 0; i < 3; i++) {
    _control_energy[i] +=
        vehicle_torque_setpoint.xyz[i] * vehicle_torque_setpoint.xyz[i] * dt;
  }

  _energy_integration_time += dt;

  if (_energy_integration_time > 500e-3f) {
    actuator_controls_status_s status;
    status.timestamp = vehicle_torque_setpoint.timestamp;

    for (int i = 0; i < 3; i++) {
      status.control_power[i] = _control_energy[i] / _energy_integration_time;
      _control_energy[i] = 0.f;
    }

    _actuator_controls_status_pub.publish(status);
    _energy_integration_time = 0.f;
  }
}

int MulticopterRateControl::task_spawn(int argc, char *argv[]) {
  bool vtol = false;

  if (argc > 1) {
    if (strcmp(argv[1], "vtol") == 0) {
      vtol = true;
    }
  }

  MulticopterRateControl *instance = new MulticopterRateControl(vtol);

  if (instance) {
    _myobject = instance;
    _object.store(instance);
    _task_id = task_id_is_work_queue;

    if (instance->init()) {
      return PX4_OK;
    }

  } else {
    PX4_ERR("alloc failed");
  }

  delete instance;
  _object.store(nullptr);
  _task_id = -1;

  return PX4_ERROR;
}

void MulticopterRateControl::do1(int loop_count) {
  PX4_INFO("=== Vehicle Thrust & Torque Setpoint Status ===");

  // Check if we're in horizontal drone mode
  const float horiz_gain = _param_mc_horiz_t2tq.get();
  if (horiz_gain > 0.0f) {
    PX4_INFO("Horizontal drone mode active (gain=%.3f)", (double)horiz_gain);
    PX4_INFO("Note: Roll/Pitch torques are converted to Y/X thrust");
  } else {
    PX4_INFO("Standard vertical drone mode");
  }

  // Show override status
  PX4_INFO("Thrust override: %s | Torque override: %s",
           _thrust_override_enabled ? "ENABLED" : "disabled",
           _torque_override_enabled ? "ENABLED" : "disabled");

  PX4_INFO("Iteration | Thrust[X,Y,Z] | Torque[Roll,Pitch,Yaw]");
  PX4_INFO("----------|---------------|------------------------");

  for (int i = 0; i < loop_count; i++) {
    // Print thrust setpoint (X, Y, Z)
    PX4_INFO("%9d | [%6.3f,%6.3f,%6.3f] | [%6.3f,%6.3f,%6.3f]", i,
             (double)_last_thrust_setpoint.xyz[0],  // X thrust
             (double)_last_thrust_setpoint.xyz[1],  // Y thrust
             (double)_last_thrust_setpoint.xyz[2],  // Z thrust
             (double)_last_torque_setpoint.xyz[0],  // Roll torque
             (double)_last_torque_setpoint.xyz[1],  // Pitch torque
             (double)_last_torque_setpoint.xyz[2]); // Yaw torque

    px4_usleep(500000); // Sleep for 500ms between iterations
  }

  PX4_INFO("=== End of Status Report ===");
}

void MulticopterRateControl::setTorque(float roll, float pitch, float yaw) {
  _torque_override(0) = roll;
  _torque_override(1) = pitch;
  _torque_override(2) = yaw;
  _torque_override_enabled = true;
  PX4_INFO("Torque override set: Roll=%.3f, Pitch=%.3f, Yaw=%.3f", (double)roll,
           (double)pitch, (double)yaw);
}

void MulticopterRateControl::setThrust(float x, float y, float z) {
  _thrust_override(0) = x;
  _thrust_override(1) = y;
  _thrust_override(2) = z;
  _thrust_override_enabled = true;
  PX4_INFO("Thrust override set: X=%.3f, Y=%.3f, Z=%.3f", (double)x, (double)y,
           (double)z);
}

void MulticopterRateControl::clearTorque() {
  _torque_override_enabled = false;
  _torque_override.zero();
  PX4_INFO("Torque override cleared");
}

void MulticopterRateControl::clearThrust() {
  _thrust_override_enabled = false;
  _thrust_override.zero();
  PX4_INFO("Thrust override cleared");
}

void MulticopterRateControl::toggleDebug() {
  _debug_enabled = !_debug_enabled;
  if (_debug_enabled) {
    mavlink_log_info(
        &_mavlink_log_pub,
        "Debug mode ENABLED - Thrust/Torque values will be printed");
    PX4_INFO("Debug mode ENABLED");
  } else {
    mavlink_log_info(&_mavlink_log_pub, "Debug mode DISABLED");
    PX4_INFO("Debug mode DISABLED");
  }
}

void MulticopterRateControl::testAxes() {
  PX4_INFO("=== Testing Individual Axes (Horizontal Drone Mode) ===");
  const float horiz_gain = _param_mc_horiz_t2tq.get();

  if (horiz_gain > 0.0f) {
    PX4_INFO("Horizontal mode active (gain=%.3f)", (double)horiz_gain);
    PX4_INFO("\nTesting each axis for 2 seconds...\n");

    // Test X thrust (forward/backward)
    PX4_INFO("Testing X thrust (forward) = 0.3...");
    setThrust(0.3f, 0.0f, 0.0f);
    px4_usleep(2000000);

    PX4_INFO("Testing X thrust (backward) = -0.3...");
    setThrust(-0.3f, 0.0f, 0.0f);
    px4_usleep(2000000);
    clearThrust();
    px4_usleep(500000);

    // Test Y thrust (left/right)
    PX4_INFO("Testing Y thrust (right) = 0.3...");
    setThrust(0.0f, 0.3f, 0.0f);
    px4_usleep(2000000);

    PX4_INFO("Testing Y thrust (left) = -0.3...");
    setThrust(0.0f, -0.3f, 0.0f);
    px4_usleep(2000000);
    clearThrust();
    px4_usleep(500000);

    // Test Yaw torque (rotation)
    PX4_INFO("Testing Yaw torque (CW) = 0.3...");
    setTorque(0.0f, 0.0f, 0.3f);
    px4_usleep(2000000);

    PX4_INFO("Testing Yaw torque (CCW) = -0.3...");
    setTorque(0.0f, 0.0f, -0.3f);
    px4_usleep(2000000);
    clearTorque();

    PX4_INFO("\n=== Motor Response Analysis ===");
    PX4_INFO("For a standard quadcopter in + configuration:");
    PX4_INFO("X thrust forward (+): Motors 3,4 should spin faster");
    PX4_INFO("X thrust backward (-): Motors 1,2 should spin faster");
    PX4_INFO("Y thrust right (+): Motors 2,4 should spin faster");
    PX4_INFO("Y thrust left (-): Motors 1,3 should spin faster");
    PX4_INFO("Yaw CW (+): Motors 1,3 faster, 2,4 slower");
    PX4_INFO("Yaw CCW (-): Motors 2,4 faster, 1,3 slower");

    PX4_INFO("\nIf Motor 1 doesn't respond to Y thrust left (-0.3),");
    PX4_INFO("check the control allocation matrix configuration.");

  } else {
    PX4_INFO("Not in horizontal mode. Set MC_HORIZ_T2TQ > 0");
  }
}
int MulticopterRateControl::custom_command(int argc, char *argv[]) {

  MulticopterRateControl *instance = _myobject;

  if (!instance) {
    PX4_ERR("Instance not running");
    return PX4_ERROR;
  }

  if (argc >= 1) {
    if (strcmp(argv[0], "do1") == 0) {
      int loop_count = 10; // Default value
      if (argc >= 2) {
        loop_count = atoi(argv[1]);
        if (loop_count <= 0) {
          PX4_WARN("Invalid loop count, using default 10");
          loop_count = 10;
        }
      }
      instance->do1(loop_count);
      return PX4_OK;
    } else if (strcmp(argv[0], "settorque") == 0) {
      if (argc >= 4) {
        float roll = strtof(argv[1], nullptr);
        float pitch = strtof(argv[2], nullptr);
        float yaw = strtof(argv[3], nullptr);
        instance->setTorque(roll, pitch, yaw);
        return PX4_OK;
      } else {
        PX4_ERR("Usage: settorque <roll> <pitch> <yaw>");
        return PX4_ERROR;
      }
    } else if (strcmp(argv[0], "setthrust") == 0) {
      if (argc >= 4) {
        float x = strtof(argv[1], nullptr);
        float y = strtof(argv[2], nullptr);
        float z = strtof(argv[3], nullptr);
        instance->setThrust(x, y, z);
        return PX4_OK;
      } else {
        PX4_ERR("Usage: setthrust <x> <y> <z>");
        return PX4_ERROR;
      }
    } else if (strcmp(argv[0], "cleartorque") == 0) {
      instance->clearTorque();
      return PX4_OK;
    } else if (strcmp(argv[0], "clearthrust") == 0) {
      instance->clearThrust();
      return PX4_OK;
    } else if (strcmp(argv[0], "testaxes") == 0) {
      instance->testAxes();
      return PX4_OK;
    } else if (strcmp(argv[0], "debug") == 0) {
      instance->toggleDebug();
      return PX4_OK;
    }
  }

  return print_usage("unknown command");
}

int MulticopterRateControl::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
Multicopter rate controller.

This module consumes rate setpoints (and body-frame thrust) from upstream
controllers (e.g., attitude controller) and drives the torque setpoints
with a PID controller.

Any horizontal-only constraints, heading hold, or manual-stick mapping
must be handled upstream.
)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  PRINT_MODULE_USAGE_COMMAND_DESCR("do1", "Print thrust/torque status");
  PRINT_MODULE_USAGE_ARG("[loop_count]", "Number of iterations (default: 10)",
                         true);

  PRINT_MODULE_USAGE_COMMAND_DESCR("settorque", "Override torque setpoint");
  PRINT_MODULE_USAGE_ARG("<roll>", "Roll torque value", false);
  PRINT_MODULE_USAGE_ARG("<pitch>", "Pitch torque value", false);
  PRINT_MODULE_USAGE_ARG("<yaw>", "Yaw torque value", false);

  PRINT_MODULE_USAGE_COMMAND_DESCR("setthrust", "Override thrust setpoint");
  PRINT_MODULE_USAGE_ARG("<x>", "X thrust value", false);
  PRINT_MODULE_USAGE_ARG("<y>", "Y thrust value", false);
  PRINT_MODULE_USAGE_ARG("<z>", "Z thrust value", false);

  PRINT_MODULE_USAGE_COMMAND_DESCR("cleartorque", "Clear torque override");
  PRINT_MODULE_USAGE_COMMAND_DESCR("clearthrust", "Clear thrust override");

  PRINT_MODULE_USAGE_COMMAND_DESCR("testaxes",
                                   "Test individual axes systematically");

  PRINT_MODULE_USAGE_COMMAND_DESCR(
      "debug", "Toggle debug mode to print thrust/torque values");

  return 0;
}

extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[]) {
  return MulticopterRateControl::main(argc, argv);
}
