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
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
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
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

// ---------- helpers ----------
static inline float wrap_pi(float a) {
  while (a > M_PI)
    a -= 2.f * M_PI;
  while (a < -M_PI)
    a += 2.f * M_PI;
  return a;
}

static inline float wrap_deg_pm180(float d) {
  while (d > 180.f)
    d -= 360.f;
  while (d < -180.f)
    d += 360.f;
  return d;
}

static inline float quat_to_yaw(const matrix::Quatf &q) {
  return Eulerf(q).psi();
}

// Per-module (singleton) state for SL heading without touching the header.
// If you prefer per-instance, move these into the class in the header.
static float g_sl_heading_rad = NAN; // target heading (rad)
static hrt_abstime g_sl_param_sync_last =
    0; // last time we pushed back to SL_HEAD_DEG

MulticopterRateControl::MulticopterRateControl(bool vtol)
    : ModuleParams(nullptr),
      WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
      _vehicle_thrust_setpoint_pub(
          vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc)
               : ORB_ID(vehicle_thrust_setpoint)),
      _vehicle_torque_setpoint_pub(
          vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc)
               : ORB_ID(vehicle_torque_setpoint)),
      _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")),
      _vehicle_attitude_sub(ORB_ID(vehicle_attitude)) // add attitude sub
{
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
  // rate control parameters
  // The controller gain K is used to convert the parallel (P + I/s + sD) form
  // to the ideal (K * [1 + 1/sTi + sTd]) form
  const Vector3f rate_k =
      Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(),
               _param_mc_yawrate_k.get());

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

  // manual rate control acro mode rate limits
  _acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()),
                            radians(_param_mc_acro_p_max.get()),
                            radians(_param_mc_acro_y_max.get()));

  _output_lpf_yaw.setCutoffFreq(_param_mc_yaw_tq_cutoff.get());

  // ---- SL heading param sanity + seed module state ----
  // Only needed if you enabled these params in the header.
  const float hdg_deg = wrap_deg_pm180(_param_sl_head_deg.get());
  _param_sl_head_deg.set(hdg_deg); // keep param bounded
  if (!PX4_ISFINITE(g_sl_heading_rad)) {
    g_sl_heading_rad = radians(hdg_deg);
  }
}

void MulticopterRateControl::Run() {
  if (should_exit()) {
    _vehicle_angular_velocity_sub.unregisterCallback();
    exit_and_cleanup();
    return;
  }

  perf_begin(_loop_perf);

  // Check if parameters have changed
  if (_parameter_update_sub.updated()) {
    // clear update
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

    // --- current yaw from attitude ---
    vehicle_attitude_s att{};
    float yaw_now = NAN;
    if (_vehicle_attitude_sub.update(&att)) {
      const Quatf q(att.q);
      yaw_now = quat_to_yaw(q);
    }

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

    // use rates setpoint topic
    vehicle_rates_setpoint_s vehicle_rates_setpoint{};

    if (_vehicle_control_mode.flag_control_manual_enabled &&
        !_vehicle_control_mode.flag_control_attitude_enabled) {
      // generate the rate setpoint from sticks
      manual_control_setpoint_s manual_control_setpoint;

      if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {

        // ===== HORIZONTAL-ONLY ATTITUDE CONTROL MODIFICATION START =====

        // Deadband to prevent drift
        const float deadband = 0.05f;
        float roll_input = (fabsf(manual_control_setpoint.roll) > deadband)
                               ? manual_control_setpoint.roll
                               : 0.0f;
        float pitch_input = (fabsf(manual_control_setpoint.pitch) > deadband)
                                ? manual_control_setpoint.pitch
                                : 0.0f;
        float yaw_input = (fabsf(manual_control_setpoint.yaw) > deadband)
                              ? manual_control_setpoint.yaw
                              : 0.0f;

        // Max tilt angles (tunable)
        const float max_roll_angle = radians(35.0f);
        const float max_pitch_angle = radians(35.0f);

        const Vector3f attitude_setpoint{roll_input * max_roll_angle,
                                         pitch_input * max_pitch_angle, 0.0f};

        // attitude -> rate mapping
        const float attitude_to_rate_gain = 4.0f;
        _rates_setpoint(0) = attitude_setpoint(0) * attitude_to_rate_gain;
        _rates_setpoint(1) = attitude_setpoint(1) * attitude_to_rate_gain;

        // ---- SL HEADING (yaw) ----
        // Seed target from param if needed
        if (!PX4_ISFINITE(g_sl_heading_rad)) {
          g_sl_heading_rad = radians(wrap_deg_pm180(_param_sl_head_deg.get()));
        }

        const bool sl_enabled = (_param_sl_head_en.get() == 1);
        const float yaw_stick_abs = fabsf(yaw_input);
        const bool pilot_yaw_active = (yaw_stick_abs > deadband);

        if (sl_enabled && PX4_ISFINITE(yaw_now)) {

          if (pilot_yaw_active) {
            // Pilot override: yaw stick directly commands yaw rate (smooth)
            const float yaw_rate_scale = 0.3f;
            _rates_setpoint(2) =
                math::superexpo(yaw_input, _param_mc_acro_expo_y.get(),
                                _param_mc_acro_supexpoy.get()) *
                _acro_rate_max(2) * yaw_rate_scale;

            // While pilot is rotating, make the SL target follow actual yaw,
            // so when the stick is released we hold the new heading.
            g_sl_heading_rad = yaw_now;

            // Push updated heading back into the param at ~5Hz (runtime value).
            const uint64_t sync_period_us = 200000; // 0.2s
            if (now - g_sl_param_sync_last > sync_period_us) {
              const float deg = wrap_deg_pm180(math::degrees(g_sl_heading_rad));
              _param_sl_head_deg.set(deg);
              g_sl_param_sync_last = now;
            }

          } else {
            // No pilot input: hold/drive to target heading using P on error
            const float kp = _param_sl_yaw_p.get(); // rad/s per rad
            const float err = wrap_pi(g_sl_heading_rad - yaw_now);
            float yaw_rate_cmd = kp * err;
            yaw_rate_cmd = math::constrain(yaw_rate_cmd, -_acro_rate_max(2),
                                           _acro_rate_max(2));
            _rates_setpoint(2) = yaw_rate_cmd;

            // Keep param value synced occasionally in case someone changed
            // target via GCS
            const uint64_t sync_period_us = 1000000; // 1s
            if (now - g_sl_param_sync_last > sync_period_us) {
              const float deg = wrap_deg_pm180(math::degrees(g_sl_heading_rad));
              _param_sl_head_deg.set(deg);
              g_sl_param_sync_last = now;
            }
          }

        } else {
          // SL disabled or yaw invalid: stick-based yaw only
          const float yaw_rate_scale = 0.3f;
          _rates_setpoint(2) =
              math::superexpo(yaw_input, _param_mc_acro_expo_y.get(),
                              _param_mc_acro_supexpoy.get()) *
              _acro_rate_max(2) * yaw_rate_scale;

          // If SL disabled but the user is rotating now and later enables SL,
          // seed target to current yaw to avoid jump.
          if (pilot_yaw_active && PX4_ISFINITE(yaw_now)) {
            g_sl_heading_rad = yaw_now;
          }
        }

        // Fixed thrust
        const float hover_thrust = -0.5f; // your fixed vertical thrust
        _thrust_setpoint(0) = 0.0f;
        _thrust_setpoint(1) = 0.0f;
        _thrust_setpoint(2) = hover_thrust;

        // ===== HORIZONTAL-ONLY ATTITUDE CONTROL MODIFICATION END =====

        // publish rate setpoint
        vehicle_rates_setpoint.roll = _rates_setpoint(0);
        vehicle_rates_setpoint.pitch = _rates_setpoint(1);
        vehicle_rates_setpoint.yaw = _rates_setpoint(2);
        _thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
        vehicle_rates_setpoint.timestamp = hrt_absolute_time();

        _vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
      }

    } else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
      if (_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) {
        _rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)
                                 ? vehicle_rates_setpoint.roll
                                 : rates(0);
        _rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch)
                                 ? vehicle_rates_setpoint.pitch
                                 : rates(1);

        // For offboard/controllers: if yaw setpoint is finite, use it.
        // Otherwise, if SL is enabled and we have yaw, use heading-hold.
        if (PX4_ISFINITE(vehicle_rates_setpoint.yaw)) {
          _rates_setpoint(2) = vehicle_rates_setpoint.yaw;
        } else if (_param_sl_head_en.get() == 1 && PX4_ISFINITE(yaw_now)) {
          if (!PX4_ISFINITE(g_sl_heading_rad)) {
            g_sl_heading_rad =
                radians(wrap_deg_pm180(_param_sl_head_deg.get()));
          }
          const float kp = _param_sl_yaw_p.get();
          const float err = wrap_pi(g_sl_heading_rad - yaw_now);
          float yaw_rate_cmd = kp * err;
          yaw_rate_cmd = math::constrain(yaw_rate_cmd, -_acro_rate_max(2),
                                         _acro_rate_max(2));
          _rates_setpoint(2) = yaw_rate_cmd;
        } else {
          _rates_setpoint(2) = rates(2);
        }

        _thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
      }
    }

    // run the rate controller
    if (_vehicle_control_mode.flag_control_rates_enabled) {

      // reset integral if disarmed
      if (!_vehicle_control_mode.flag_armed ||
          _vehicle_status.vehicle_type !=
              vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
        _rate_control.resetIntegral();
      }

      // update saturation status from control allocation feedback
      control_allocator_status_s control_allocator_status;

      if (_control_allocator_status_sub.update(&control_allocator_status)) {
        Vector<bool, 3> saturation_positive;
        Vector<bool, 3> saturation_negative;

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

      // publish rate controller status
      rate_ctrl_status_s rate_ctrl_status{};
      _rate_control.getRateControlStatus(rate_ctrl_status);
      rate_ctrl_status.timestamp = hrt_absolute_time();
      _controller_status_pub.publish(rate_ctrl_status);

      // publish thrust and torque setpoints
      vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
      vehicle_torque_setpoint_s vehicle_torque_setpoint{};

      _thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);

      // Enable all 3 torque axes
      vehicle_torque_setpoint.xyz[0] =
          PX4_ISFINITE(torque_setpoint(0)) ? torque_setpoint(0) : 0.f;
      vehicle_torque_setpoint.xyz[1] =
          PX4_ISFINITE(torque_setpoint(1)) ? torque_setpoint(1) : 0.f;
      vehicle_torque_setpoint.xyz[2] =
          PX4_ISFINITE(torque_setpoint(2)) ? torque_setpoint(2) : 0.f;

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

      vehicle_thrust_setpoint.timestamp_sample =
          angular_velocity.timestamp_sample;
      vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
      _vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

      vehicle_torque_setpoint.timestamp_sample =
          angular_velocity.timestamp_sample;
      vehicle_torque_setpoint.timestamp = hrt_absolute_time();
      _vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

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

int MulticopterRateControl::custom_command(int argc, char *argv[]) {
  return print_usage("unknown command");
}

int MulticopterRateControl::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
This implements the multicopter rate controller modified for horizontal-only movement
using attitude-based control for stabilization and SL heading hold.

HORIZONTAL-ONLY MODIFICATIONS:
- Roll and pitch stick inputs generate attitude setpoints for horizontal movement
- Attitude controller provides automatic stabilization when sticks are centered
- Vertical thrust is fixed for altitude hold (tethered constraint)
- Yaw rate control provides continuous rotation capability
- SL heading: when enabled, yaw holds a parameterized heading. Manual yaw temporarily overrides and updates the target.

The controller uses PID loops for all three axes with attitude setpoint conversion
for roll/pitch (horizontal stabilization) and yaw heading-hold or manual yaw.

)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}

extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[]) {
  return MulticopterRateControl::main(argc, argv);
}
