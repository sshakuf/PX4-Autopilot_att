/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

using namespace matrix;
using math::constrain;

const trajectory_setpoint_s PositionControl::empty_trajectory_setpoint = {
    0,  {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN,
    NAN};

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I,
                                       const Vector3f &D) {
  _gain_vel_p = P;
  _gain_vel_i = I;
  _gain_vel_d = D;
}

void PositionControl::setVelocityLimits(const float vel_horizontal,
                                        const float vel_up,
                                        const float vel_down) {
  _lim_vel_horizontal = vel_horizontal;
  _lim_vel_up = vel_up;
  _lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max) {
  // make sure there's always enough thrust vector length to infer the attitude
  _lim_thr_min = math::max(min, 10e-4f);
  _lim_thr_max = max;
}

void PositionControl::setHorizontalThrustMargin(const float margin) {
  _lim_thr_xy_margin = margin;
}

void PositionControl::updateHoverThrust(const float hover_thrust_new) {
  // Given that the equation for thrust is T = a_sp * Th / g - Th
  // with a_sp = desired acceleration, Th = hover thrust and g = gravity
  // constant, we want to find the acceleration that needs to be added to the
  // integrator in order obtain the same thrust after replacing the current
  // hover thrust by the new one. T' = T => a_sp' * Th' / g - Th' = a_sp * Th /
  // g - Th so a_sp' = (a_sp - g) * Th / Th' + g we can then add a_sp' - a_sp to
  // the current integrator to absorb the effect of changing Th by Th'
  const float previous_hover_thrust = _hover_thrust;
  setHoverThrust(hover_thrust_new);

  _vel_int(2) +=
      (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust +
      CONSTANTS_ONE_G - _acc_sp(2);
}

void PositionControl::setState(const PositionControlStates &states) {
  _pos = states.position;
  _vel = states.velocity;
  _yaw = states.yaw;
  _vel_dot = states.acceleration;
}

void PositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint) {
  _pos_sp = Vector3f(setpoint.position);
  _vel_sp = Vector3f(setpoint.velocity);
  _acc_sp = Vector3f(setpoint.acceleration);
  _yaw_sp = setpoint.yaw;
  _yawspeed_sp = setpoint.yawspeed;
}

bool PositionControl::update(const float dt) {
  bool valid = _inputValid();

  if (valid) {
    _positionControl();
    _velocityControl(dt);

    _yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
    _yaw_sp = PX4_ISFINITE(_yaw_sp)
                  ? _yaw_sp
                  : _yaw; // TODO: better way to disable yaw control
  }

  // There has to be a valid output acceleration and thrust setpoint otherwise
  // something went wrong
  return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}

void PositionControl::_positionControl() {
  // P-position controller - Horizontal-only drone modification
  Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);

  // Debug: Enhanced logging for position control
  static int debug_counter = 0;
  if (++debug_counter >= 50) { // Print every 50 cycles
    debug_counter = 0;

    // Log position errors and gains
    Vector2f pos_error = _pos_sp.xy() - _pos.xy();
    PX4_INFO("[POS_CTRL] Pos SP[%.3f,%.3f] - Pos[%.3f,%.3f] = Error[%.3f,%.3f]",
             PX4_ISFINITE(_pos_sp(0)) ? (double)_pos_sp(0) : (double)NAN,
             PX4_ISFINITE(_pos_sp(1)) ? (double)_pos_sp(1) : (double)NAN,
             PX4_ISFINITE(_pos(0)) ? (double)_pos(0) : (double)NAN,
             PX4_ISFINITE(_pos(1)) ? (double)_pos(1) : (double)NAN,
             (double)pos_error(0), (double)pos_error(1));

    PX4_INFO("[POS_CTRL] Gain P[%.2f,%.2f] -> Vel cmd[%.3f,%.3f]",
             (double)_gain_pos_p(0), (double)_gain_pos_p(1),
             (double)vel_sp_position(0), (double)vel_sp_position(1));
  }

  // Position and feed-forward velocity setpoints or position states being NAN
  // results in them not having an influence
  ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
  // make sure there are no NAN elements for further reference while
  // constraining
  ControlMath::setZeroIfNanVector3f(vel_sp_position);

  // Constrain horizontal velocity by prioritizing the velocity component along
  // the the desired position setpoint over the feed-forward term.
  _vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(),
                                          (_vel_sp - vel_sp_position).xy(),
                                          _lim_vel_horizontal);

  // Horizontal-only drone: No vertical velocity control
  _vel_sp(2) = 0.0f;
}

void PositionControl::_velocityControl(const float dt) {
  // Horizontal-only drone: No vertical velocity integral constraint needed

  // PID velocity control for horizontal axes only
  Vector3f vel_error = _vel_sp - _vel;

  // Zero out vertical velocity error for horizontal-only drone
  vel_error(2) = 0.0f;

  // Debug: Log velocity control
  static int vel_debug_counter = 0;
  if (++vel_debug_counter >= 50) {
    vel_debug_counter = 0;
    PX4_INFO("[VEL_CTRL] Vel SP[%.3f,%.3f] - Vel[%.3f,%.3f] = Error[%.3f,%.3f]",
             (double)_vel_sp(0), (double)_vel_sp(1),
             (double)_vel(0), (double)_vel(1),
             (double)vel_error(0), (double)vel_error(1));
  }

  // Compute acceleration setpoint from velocity error (horizontal only)
  Vector3f acc_sp_velocity =
      vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

  // Force zero vertical acceleration for horizontal-only drone
  acc_sp_velocity(2) = 0.0f;

  // Debug: Log PID components
  if (vel_debug_counter == 1) {  // Print right after the main velocity debug
    PX4_INFO("[VEL_PID] P[%.3f,%.3f] I[%.3f,%.3f] D[%.3f,%.3f] -> Acc[%.3f,%.3f]",
             (double)(vel_error(0) * _gain_vel_p(0)),
             (double)(vel_error(1) * _gain_vel_p(1)),
             (double)_vel_int(0), (double)_vel_int(1),
             (double)(-_vel_dot(0) * _gain_vel_d(0)),
             (double)(-_vel_dot(1) * _gain_vel_d(1)),
             (double)acc_sp_velocity(0), (double)acc_sp_velocity(1));
  }

  // No control input from setpoints or corresponding states which are NAN
  ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

  // Ensure no vertical acceleration
  _acc_sp(2) = 0.0f;

  _accelerationControl();

  // Horizontal thrust saturation only
  const Vector2f thrust_sp_xy(_thr_sp);
  const float thrust_sp_xy_norm = thrust_sp_xy.norm();

  // Saturate thrust in horizontal direction
  if (thrust_sp_xy_norm > _lim_thr_max) {
    _thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * _lim_thr_max;
  }

  // Use tracking Anti-Windup for horizontal direction
  const Vector2f acc_sp_xy_produced =
      Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);

  // The ARW loop needs to run if the signal is saturated only.
  if (_acc_sp.xy().norm_squared() > acc_sp_xy_produced.norm_squared()) {
    const float arw_gain = 2.f / _gain_vel_p(0);
    const Vector2f acc_sp_xy = _acc_sp.xy();

    vel_error.xy() =
        Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_sp_xy_produced);
  }

  // Make sure integral doesn't get NAN
  ControlMath::setZeroIfNanVector3f(vel_error);
  // Update integral part of velocity control (horizontal only)
  _vel_int += vel_error.emult(_gain_vel_i) * dt;

  // Keep vertical integral at zero
  _vel_int(2) = 0.0f;
}

void PositionControl::_accelerationControl() {
  // Horizontal-only drone: Direct thrust control without attitude tilting
  // Convert desired horizontal acceleration directly to thrust

  // Direct mapping from acceleration to thrust
  // For horizontal movement, we need significant thrust to overcome inertia
  // Scale the acceleration to thrust with a reasonable gain

  // Normalize accelerations to thrust range [-1, 1]
  // Reduced max acceleration for more aggressive response
  // Lower value = stronger thrust for same acceleration demand
  const float max_horizontal_acc =
      2.0f; // Reduced from 5.0 for stronger response

  _thr_sp(0) = math::constrain(_acc_sp(0) / max_horizontal_acc, -1.0f,
                               1.0f); // Fx - forward/backward thrust
  _thr_sp(1) = math::constrain(_acc_sp(1) / max_horizontal_acc, -1.0f,
                               1.0f); // Fy - left/right thrust
  _thr_sp(2) = 0.0f;                  // Fz - No vertical thrust capability

  // Apply horizontal thrust limits
  Vector2f thrust_xy(_thr_sp);
  float thrust_xy_mag = thrust_xy.norm();

  // Limit maximum horizontal thrust to prevent saturation
  if (thrust_xy_mag > 1.0f) {
    _thr_sp.xy() = thrust_xy.normalized();
  }

  // Debug: Enhanced acceleration to thrust conversion logging
  static int thrust_debug_counter = 0;
  if (++thrust_debug_counter >= 50) { // Print every 50 cycles
    thrust_debug_counter = 0;
    PX4_INFO("[ACC_CTRL] Acc SP[%.3f,%.3f,%.3f] -> Thrust[%.3f,%.3f,%.3f]",
             (double)_acc_sp(0), (double)_acc_sp(1), (double)_acc_sp(2),
             (double)_thr_sp(0), (double)_thr_sp(1), (double)_thr_sp(2));

    if (thrust_xy_mag > 0.01f) {
      PX4_INFO("[ACC_CTRL] Thrust magnitude: %.3f (limited: %s)",
               (double)thrust_xy_mag, thrust_xy_mag > 1.0f ? "YES" : "NO");
    }
  }
}

bool PositionControl::_inputValid() {
  bool valid = true;

  // Every axis x, y, z needs to have some setpoint
  for (int i = 0; i <= 2; i++) {
    valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) ||
                      PX4_ISFINITE(_acc_sp(i)));
  }

  // x and y input setpoints always have to come in pairs
  valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
  valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
  valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

  // For each controlled state the estimate has to be valid
  for (int i = 0; i <= 2; i++) {
    if (PX4_ISFINITE(_pos_sp(i))) {
      valid = valid && PX4_ISFINITE(_pos(i));
    }

    if (PX4_ISFINITE(_vel_sp(i))) {
      valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
    }
  }

  return valid;
}

void PositionControl::getLocalPositionSetpoint(
    vehicle_local_position_setpoint_s &local_position_setpoint) const {
  local_position_setpoint.x = _pos_sp(0);
  local_position_setpoint.y = _pos_sp(1);
  local_position_setpoint.z = _pos_sp(2);
  local_position_setpoint.yaw = _yaw_sp;
  local_position_setpoint.yawspeed = _yawspeed_sp;
  local_position_setpoint.vx = _vel_sp(0);
  local_position_setpoint.vy = _vel_sp(1);
  local_position_setpoint.vz = _vel_sp(2);
  _acc_sp.copyTo(local_position_setpoint.acceleration);
  _thr_sp.copyTo(local_position_setpoint.thrust);
}

void PositionControl::getAttitudeSetpoint(
    vehicle_attitude_setpoint_s &attitude_setpoint) const {
  // Horizontal-only drone: Output direct thrust commands without attitude
  // tilting Maintain level attitude (no roll/pitch) and pass horizontal thrust
  // directly

  // Set level attitude - no tilting for horizontal movement
  Quaternionf q_sp;
  q_sp = Quaternionf(
      Eulerf(0.0f, 0.0f, _yaw_sp)); // Roll=0, Pitch=0, Yaw=controlled
  q_sp.copyTo(attitude_setpoint.q_d);

  // Pass the direct thrust commands
  // thrust_body[0] = forward/backward thrust (body frame X)
  // thrust_body[1] = left/right thrust (body frame Y)
  // thrust_body[2] = up/down thrust (always 0 for horizontal-only)
  attitude_setpoint.thrust_body[0] = _thr_sp(0);
  attitude_setpoint.thrust_body[1] = _thr_sp(1);
  attitude_setpoint.thrust_body[2] = 0.0f;

  // Set yaw rate
  attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
