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
 * @file PositionControl.hpp
 *
 * A cascaded position controller for position/velocity control only.
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

struct PositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
	float yaw_rate;
};

/**
 * 	Core Position-Control for MC.
 * 	This class contains P-controller for position and
 * 	PID-controller for velocity.
 * 	Inputs:
 * 		vehicle position/velocity/yaw
 * 		desired set-point position/velocity/thrust/yaw/yaw-speed
 * 		constraints that are stricter than global limits
 * 	Output
 * 		thrust vector and a yaw-setpoint
 *
 * 	If there is a position and a velocity set-point present, then
 * 	the velocity set-point is used as feed-forward. If feed-forward is
 * 	active, then the velocity component of the P-controller output has
 * 	priority over the feed-forward component.
 *
 * 	A setpoint that is NAN is considered as not set.
 * 	If there is a position/velocity- and thrust-setpoint present, then
 *  the thrust-setpoint is ommitted and recomputed from position-velocity-PID-loop.
 */
class PositionControl
{
public:

	PositionControl() = default;
	~PositionControl() = default;

	/**
	 * Set the position control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 */
	void setPositionGains(const matrix::Vector3f &P) { _gain_pos_p = P; }

	/**
	 * Set the velocity control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setVelocityGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/**
	 * Set the maximum velocity to execute with feed forward and position control
	 * @param vel_horizontal horizontal velocity limit
	 * @param vel_up not used for horizontal-only drone
	 * @param vel_down not used for horizontal-only drone
	 */
	void setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down);

	/**
	 * Set the minimum and maximum collective normalized thrust [0,1] that can be output by the controller
	 * @param min minimum thrust e.g. 0.1 or 0
	 * @param max maximum thrust e.g. 0.9 or 1
	 */
	void setThrustLimits(const float min, const float max);

	/**
	 * Set margin for horizontal thrust - not used for horizontal-only drone
	 * @param margin of normalized thrust
	 */
	void setHorizontalThrustMargin(const float margin);

	/**
	 * Set the maximum tilt angle - not used for horizontal-only drone (always level)
	 * @param tilt angle in radians from level orientation
	 */
	void setTiltLimit(const float tilt) { _lim_tilt = 0.0f; /* Always level */ }

	/**
	 * Set the normalized hover thrust
	 * @param hover_thrust [HOVER_THRUST_MIN, HOVER_THRUST_MAX] with which the vehicle hovers not accelerating down or up with level orientation
	 */
	void setHoverThrust(const float hover_thrust) { _hover_thrust = math::constrain(hover_thrust, HOVER_THRUST_MIN, HOVER_THRUST_MAX); }

	/**
	 * Update the hover thrust without immediately affecting the output
	 * by adjusting the integrator. This prevents propagating the dynamics
	 * of the hover thrust signal directly to the output of the controller.
	 */
	void updateHoverThrust(const float hover_thrust_new);

	/**
	 * Set keep heading parameters
	 * @param enable Enable/disable keep heading feature
	 * @param heading_deg Target heading in degrees (0-360, 0=North)
	 */
	void setKeepHeading(bool enable, float heading_deg);

	/**
	 * Enable/disable the outer yaw-speed PID (keep-heading PID).
	 * When false, yaw-speed setpoint is forced to 0 and integrators reset,
	 * even when keep_heading is otherwise enabled. For diagnostics.
	 */
	void setYawSpeedPidEnabled(bool enable) { _yawspeed_pid_enabled = enable; }

	/**
	 * Set maximum yaw rate for keep heading
	 * @param max_yaw_rate_deg_s Maximum yaw rate in degrees per second
	 */
	void setMaxYawRate(float max_yaw_rate_deg_s);

	/**
	 * Set the yaw speed PID gains
	 * @param P heading error to yaw-rate gain
	 * @param I heading integral to yaw-rate gain
	 * @param D yaw-rate damping gain
	 */
	void setYawSpeedGains(float P, float I, float D);

	/**
	 * Set the fine heading hold parameters used near the target heading
	 * @param error_deg heading error threshold for fine mode in degrees
	 * @param rate_limit_deg_s fine mode heading correction rate limit in degrees per second
	 * @param P fine heading error to yaw-rate gain
	 * @param I fine heading integral to yaw-rate gain
	 * @param D fine yaw-rate damping gain
	 * @param integral_limit_deg_s fine integral yaw-rate contribution limit in degrees per second
	 * @param brake_accel_deg_s2 estimated fine-mode yaw braking acceleration in degrees per second squared
	 * @param tolerance_deg acceptable fine heading error in degrees
	 * @param min_rate_deg_s minimum fine correction yaw rate outside tolerance in degrees per second
	 */
	void setFineYawSpeedGains(float error_deg, float rate_limit_deg_s, float P, float I, float D,
				  float integral_limit_deg_s, float brake_accel_deg_s2, float tolerance_deg,
				  float min_rate_deg_s);

	/**
	 * Set maximum yaw acceleration for keep heading
	 * @param max_yaw_accel_deg_s2 Maximum yaw acceleration in degrees per second squared
	 */
	void setMaxYawAcceleration(float max_yaw_accel_deg_s2);

	/**
	 * Pass the current vehicle state to the controller
	 * @param PositionControlStates structure
	 */
	void setState(const PositionControlStates &states);

	/**
	 * Pass the desired setpoints
	 * Note: NAN value means no feed forward/leave state uncontrolled if there's no higher order setpoint.
	 * @param setpoint setpoints including feed-forwards to execute in update()
	 */
	void setInputSetpoint(const trajectory_setpoint_s &setpoint);

	/**
	 * Relax position/velocity requirement when DF_POS_RELAX is set (horizontal drone).
	 * When true, controller accepts acceleration-only setpoints without valid estimator position/velocity.
	 */
	void setPositionRelaxed(bool relaxed) { _position_relaxed = relaxed; }

	/**
	 * Apply P-position and PID-velocity controller that updates the member
	 * thrust, yaw- and yawspeed-setpoints.
	 * @see _thr_sp
	 * @see _yaw_sp
	 * @see _yawspeed_sp
	 * @param dt time in seconds since last iteration
	 * @return true if update succeeded and output setpoint is executable, false if not
	 */
	bool update(const float dt);

	/**
	 * Set the integral term in xy to 0.
	 * @see _vel_int
	 */
	void resetIntegral() { _vel_int.setZero(); }
	void resetIntegralXY() { _vel_int.xy() = matrix::Vector2f(); }

	/**
	 * Not used for horizontal-only drone
	 */
	void decoupleHorizontalAndVecticalAcceleration(bool val) { /* Not used */ }

	/**
	 * Get the controllers output local position setpoint
	 * These setpoints are the ones which were executed on including PID output and feed-forward.
	 * The acceleration or thrust setpoints can be used for attitude control.
	 * @param local_position_setpoint reference to struct to fill up
	 */
	void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;

	/**
	 * Get the controllers output attitude setpoint
	 * This attitude setpoint was generated from the resulting acceleration setpoint after position and velocity control.
	 * It needs to be executed by the attitude controller to achieve velocity and position tracking.
	 * @param attitude_setpoint reference to struct to fill up
	 */
	void getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const;

	/**
	 * Get the direct thrust setpoint for horizontal-only operation
	 * @param thrust_sp reference to thrust vector to fill
	 */
	void getThrustSetpoint(matrix::Vector3f &thrust_sp) const { thrust_sp = _thr_sp; }

	/**
	 * Get the yaw rate setpoint
	 * @return yaw rate setpoint in rad/s
	 */
	float getYawRateSetpoint() const { return _yawspeed_sp; }

	/**
	 * All setpoints are set to NAN (uncontrolled). Timestampt zero.
	 */
	static const trajectory_setpoint_s empty_trajectory_setpoint;

private:
	// The range limits of the hover thrust configuration/estimate
	static constexpr float HOVER_THRUST_MIN = 0.05f;
	static constexpr float HOVER_THRUST_MAX = 0.9f;

	bool _inputValid();

	void _positionControl(); ///< Position proportional control
	void _velocityControl(const float dt); ///< Velocity PID control
	void _accelerationControl(); ///< Acceleration setpoint processing

	// Gains
	matrix::Vector3f _gain_pos_p; ///< Position control proportional gain
	matrix::Vector3f _gain_vel_p; ///< Velocity control proportional gain
	matrix::Vector3f _gain_vel_i; ///< Velocity control integral gain
	matrix::Vector3f _gain_vel_d; ///< Velocity control derivative gain

	// Limits
	float _lim_vel_horizontal{}; ///< Horizontal velocity limit with feed forward and position control
	float _lim_vel_up{}; ///< Upwards velocity limit with feed forward and position control
	float _lim_vel_down{}; ///< Downwards velocity limit with feed forward and position control
	float _lim_thr_min{}; ///< Minimum collective thrust allowed as output [-1,0] e.g. -0.9
	float _lim_thr_max{}; ///< Maximum collective thrust allowed as output [-1,0] e.g. -0.1
	float _lim_thr_xy_margin{}; ///< Margin to keep for horizontal control when saturating prioritized vertical thrust
	float _lim_tilt{}; ///< Maximum tilt from level the output attitude is allowed to have

	float _hover_thrust{}; ///< Thrust [HOVER_THRUST_MIN, HOVER_THRUST_MAX] with which the vehicle hovers not accelerating down or up with level orientation

	// States
	matrix::Vector3f _pos; /**< current position */
	matrix::Vector3f _vel; /**< current velocity */
	matrix::Vector3f _vel_dot; /**< velocity derivative (replacement for acceleration estimate) */
	matrix::Vector3f _vel_int; /**< integral term of the velocity controller */
	float _yaw{}; /**< current heading */

	// Setpoints
	matrix::Vector3f _pos_sp; /**< desired position */
	matrix::Vector3f _vel_sp; /**< desired velocity */
	matrix::Vector3f _acc_sp; /**< desired acceleration */
	matrix::Vector3f _thr_sp; /**< desired thrust */
	float _yaw_sp{}; /**< desired heading */
	float _yawspeed_sp{}; /** desired yaw-speed */

	// Keep heading feature
	bool _keep_heading_enabled{false}; /**< enable keep heading feature */
	bool _yawspeed_pid_enabled{true}; /**< enable outer yaw-speed PID (DF_YAWSPEED_PID_EN) */
	float _keep_heading_target{0.0f}; /**< target heading in radians */
	float _max_yaw_rate{math::radians(20.0f)}; /**< maximum yaw rate in rad/s */
	float _max_yaw_accel{math::radians(10.0f)}; /**< maximum yaw acceleration in rad/s^2 */

	// Keep-heading yaw-rate shaping gains
	float _gain_yawspeed_p{0.8f}; /**< heading error to yaw-rate gain */
	float _gain_yawspeed_i{0.08f}; /**< heading integral to yaw-rate gain */
	float _gain_yawspeed_d{0.8f}; /**< yaw-rate damping gain */

	float _fine_yaw_error{math::radians(25.0f)}; /**< heading error threshold for fine mode */
	float _fine_yaw_rate_limit{math::radians(35.0f)}; /**< fine mode heading correction yaw-rate limit */
	float _fine_yawspeed_p{2.0f}; /**< fine heading error to yaw-rate gain */
	float _fine_yawspeed_i{0.12f}; /**< fine heading integral to yaw-rate gain */
	float _fine_yawspeed_d{1.2f}; /**< fine yaw-rate damping gain */
	float _fine_yawspeed_ilim{math::radians(12.0f)}; /**< fine integral yaw-rate contribution limit */
	float _fine_yaw_brake_accel{math::radians(20.0f)}; /**< estimated fine-mode yaw braking acceleration */
	float _fine_yaw_tolerance{math::radians(3.0f)}; /**< acceptable fine heading error */
	float _fine_yaw_min_rate{math::radians(12.0f)}; /**< minimum fine correction yaw rate outside tolerance */

	// Keep-heading yaw-rate shaping state
	float _yawspeed_error_prev{0.0f}; /**< kept for API compatibility with older tuning code */
	float _yawspeed_integral{0.0f}; /**< heading error integral accumulator */
	float _yawspeed_sp_prev{0.0f}; /**< previous yaw speed setpoint for acceleration limiting */
	float _yaw_rate{0.0f}; /**< current gyro yaw rate (rad/s) */

	bool _position_relaxed{false}; /**< DF_POS_RELAX: accept acc_sp without valid pos/vel from estimator */
};
