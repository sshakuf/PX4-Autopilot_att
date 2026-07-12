/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file HeadingHoldControl.hpp
 *
 * Keep-heading (DF_YAW_*) yaw-rate controller shared between the position
 * controller and the attitude controller. Converts a fixed target heading
 * into a yaw-rate setpoint using a coarse turn profile plus an optional
 * fine heading-hold PID near the target.
 */

#pragma once

#include <lib/mathlib/mathlib.h>

class HeadingHoldControl
{
public:
	HeadingHoldControl() = default;
	~HeadingHoldControl() = default;

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
	 * Enable/disable the fine heading-hold stage (DF_YAW_FINE_EN).
	 * When false, the fine PID is bypassed and the coarse yaw-speed PID
	 * (DF_YAWSPEED_P/I/D) is used across the whole heading-error range.
	 */
	void setFineYawEnabled(bool enable) { _fine_yaw_enabled = enable; }

	/**
	 * Set maximum yaw rate for keep heading
	 * @param max_yaw_rate_deg_s Maximum yaw rate in degrees per second
	 */
	void setMaxYawRate(float max_yaw_rate_deg_s);

	/**
	 * Set maximum yaw acceleration for keep heading
	 * @param max_yaw_accel_deg_s2 Maximum yaw acceleration in degrees per second squared
	 */
	void setMaxYawAcceleration(float max_yaw_accel_deg_s2);

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

	bool enabled() const { return _keep_heading_enabled; }

	/** Target heading in radians, wrapped to [-pi, pi] */
	float targetHeading() const { return _keep_heading_target; }

	/**
	 * Run the keep-heading controller
	 * @param yaw current heading in radians
	 * @param yaw_rate current gyro yaw rate in rad/s
	 * @param dt time in seconds since last iteration
	 * @return yaw-rate setpoint in rad/s
	 */
	float update(float yaw, float yaw_rate, float dt);

	/** Reset the PID state (integral, slew memory) */
	void resetState();

private:
	bool _keep_heading_enabled{false}; /**< enable keep heading feature */
	bool _yawspeed_pid_enabled{true}; /**< enable outer yaw-speed PID (DF_YAWSPD_PID_EN) */
	bool _fine_yaw_enabled{true}; /**< enable fine heading-hold stage (DF_YAW_FINE_EN); false = coarse PID only */
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
};
