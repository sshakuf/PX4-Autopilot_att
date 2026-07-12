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
 * @file HeadingHoldControl.cpp
 */

#include "HeadingHoldControl.hpp"

#include <float.h>
#include <matrix/matrix/math.hpp>

using namespace matrix;

void HeadingHoldControl::setKeepHeading(bool enable, float heading_deg)
{
	if (!enable || !_keep_heading_enabled) {
		resetState();
	}

	_keep_heading_enabled = enable;
	// Convert degrees to radians and normalize to [-pi, pi]
	// Heading is in NED coordinates: 0° = North, 90° = East, 180° = South, -90°/270° = West
	_keep_heading_target = math::radians(heading_deg);
	_keep_heading_target = wrap_pi(_keep_heading_target);
}

void HeadingHoldControl::setMaxYawRate(float max_yaw_rate_deg_s)
{
	_max_yaw_rate = math::radians(max_yaw_rate_deg_s);
}

void HeadingHoldControl::setMaxYawAcceleration(float max_yaw_accel_deg_s2)
{
	_max_yaw_accel = math::radians(max_yaw_accel_deg_s2);
}

void HeadingHoldControl::setYawSpeedGains(float P, float I, float D)
{
	_gain_yawspeed_p = P;
	_gain_yawspeed_i = I;
	_gain_yawspeed_d = D;
}

void HeadingHoldControl::setFineYawSpeedGains(float error_deg, float rate_limit_deg_s, float P, float I, float D,
		float integral_limit_deg_s, float brake_accel_deg_s2, float tolerance_deg,
		float min_rate_deg_s)
{
	_fine_yaw_error = math::radians(error_deg);
	_fine_yaw_rate_limit = math::radians(rate_limit_deg_s);
	_fine_yawspeed_p = P;
	_fine_yawspeed_i = I;
	_fine_yawspeed_d = D;
	_fine_yawspeed_ilim = math::radians(integral_limit_deg_s);
	_fine_yaw_brake_accel = math::radians(brake_accel_deg_s2);
	_fine_yaw_tolerance = math::radians(tolerance_deg);
	_fine_yaw_min_rate = math::radians(min_rate_deg_s);
}

void HeadingHoldControl::resetState()
{
	_yawspeed_integral = 0.0f;
	_yawspeed_error_prev = 0.0f;
	_yawspeed_sp_prev = 0.0f;
}

float HeadingHoldControl::update(const float yaw, const float yaw_rate, const float dt)
{
	if (!_yawspeed_pid_enabled) {
		// DF_YAWSPD_PID_EN=0: bypass outer yaw-speed PID and command zero
		// yaw-rate so the inner rate loop simply holds yaw rate at 0.
		resetState();
		return 0.f;
	}

	const float dt_limited = math::constrain(dt, 0.002f, 0.04f);
	const float max_yaw_rate = math::max(_max_yaw_rate, math::radians(1.0f));
	const float max_yaw_accel = math::max(_max_yaw_accel, math::radians(1.0f));

	const float heading_error = wrap_pi(_keep_heading_target - yaw);
	const float abs_heading_error = fabsf(heading_error);
	const float direction = heading_error >= 0.0f ? 1.0f : -1.0f;
	const bool rotating_towards_target = yaw_rate * direction > 0.0f;
	const float fine_yaw_error = math::max(_fine_yaw_error, math::radians(0.5f));
	const float fine_yaw_brake_accel = math::max(_fine_yaw_brake_accel, math::radians(1.0f));
	const float stopping_angle = yaw_rate * fabsf(yaw_rate) / (2.0f * fine_yaw_brake_accel);
	const float compensated_heading_error = heading_error - stopping_angle;
	// DF_YAW_FINE_EN=0 disables the fine stage entirely: keeping this false
	// makes every downstream branch (rate, integral, damping, slew) fall back
	// to the coarse yaw-speed PID across the whole heading-error range.
	const bool fine_heading_hold = _fine_yaw_enabled
				       && ((abs_heading_error < fine_yaw_error)
					   || (rotating_towards_target && (fabsf(compensated_heading_error) < fine_yaw_error)));

	// Braking profile: maximum rate that can still stop inside the remaining angle.
	const float stopping_limited_rate = sqrtf(2.0f * max_yaw_accel * abs_heading_error);
	const float heading_rate = _gain_yawspeed_p * abs_heading_error;
	const float profile_yaw_rate_sp = direction * math::min(max_yaw_rate, math::min(heading_rate, stopping_limited_rate));
	float yaw_rate_sp = profile_yaw_rate_sp;

	if (fine_heading_hold) {
		const float fine_heading_rate_limit = math::min(max_yaw_rate,
						      math::max(_fine_yaw_rate_limit, math::radians(1.0f)));
		const float fine_heading_rate = math::constrain(_fine_yawspeed_p * compensated_heading_error,
					        -fine_heading_rate_limit,
					        fine_heading_rate_limit);

		yaw_rate_sp = fine_heading_rate - _fine_yawspeed_d * yaw_rate;

		const float fine_min_rate = math::min(fine_heading_rate_limit, math::max(_fine_yaw_min_rate, 0.0f));

		const float drift_arrest_error = math::max(3.0f * _fine_yaw_tolerance, math::radians(2.0f));
		const bool drifting_away_from_target = yaw_rate * heading_error < 0.0f;

		if ((fine_min_rate > FLT_EPSILON)
		    && (abs_heading_error < drift_arrest_error)
		    && drifting_away_from_target
		    && (fabsf(yaw_rate) > math::radians(0.5f))
		    && (fabsf(yaw_rate_sp) < fine_min_rate)) {
			const float drift_arrest_rate = math::constrain(-2.0f * _fine_yawspeed_d * yaw_rate,
						        -fine_min_rate, fine_min_rate);

			if (fabsf(drift_arrest_rate) > fabsf(yaw_rate_sp)) {
				yaw_rate_sp = drift_arrest_rate;
			}

		} else if ((fine_min_rate > FLT_EPSILON)
			   && (abs_heading_error > math::radians(0.25f))
			   && (fabsf(yaw_rate) < math::radians(5.0f))) {
			const float boost_scale = math::constrain(abs_heading_error / math::max(_fine_yaw_tolerance, math::radians(0.5f)),
						  0.0f, 1.0f);
			const float boosted_yaw_rate_sp = yaw_rate_sp + direction * fine_min_rate * boost_scale;
			yaw_rate_sp = math::constrain(boosted_yaw_rate_sp, -fine_heading_rate_limit, fine_heading_rate_limit);
		}
	}

	const float yaw_i_gain = fine_heading_hold ? _fine_yawspeed_i : _gain_yawspeed_i;

	if (yaw_i_gain > FLT_EPSILON) {
		const float integral_yaw_rate_limit = fine_heading_hold ? _fine_yawspeed_ilim : 0.25f * max_yaw_rate;
		const float integral_limit = integral_yaw_rate_limit / yaw_i_gain;
		const float yaw_rate_i = yaw_i_gain * _yawspeed_integral;
		const float yaw_rate_unsaturated = yaw_rate_sp + yaw_rate_i;
		const bool saturated = fabsf(yaw_rate_unsaturated) >= max_yaw_rate;

		// Integrate only when not driving deeper into yaw-rate saturation.
		if (fine_heading_hold && (!saturated || (heading_error * yaw_rate_unsaturated < 0.0f))) {
			_yawspeed_integral = math::constrain(_yawspeed_integral + heading_error * dt_limited,
							     -integral_limit, integral_limit);

		} else if (!fine_heading_hold) {
			_yawspeed_integral *= 0.98f;
		}

		yaw_rate_sp += yaw_i_gain * _yawspeed_integral;

	} else {
		_yawspeed_integral = 0.0f;
	}

	// Brake only when the vehicle is already rotating faster than the
	// stopping profile allows. Do not damp normal motion toward the target.
	const float excess_yaw_rate = fabsf(yaw_rate) - fabsf(profile_yaw_rate_sp);

	if (!fine_heading_hold && rotating_towards_target && excess_yaw_rate > 0.0f) {
		yaw_rate_sp -= direction * _gain_yawspeed_d * excess_yaw_rate;
	}

	yaw_rate_sp = math::constrain(yaw_rate_sp, -max_yaw_rate, max_yaw_rate);

	const bool setpoint_moves_towards_target_faster = yaw_rate_sp * direction > _yawspeed_sp_prev * direction;

	float yawspeed_sp;

	if (fine_heading_hold) {
		const float max_delta_yaw_rate = fine_yaw_brake_accel * dt_limited;
		yawspeed_sp = _yawspeed_sp_prev + math::constrain(yaw_rate_sp - _yawspeed_sp_prev,
				-max_delta_yaw_rate, max_delta_yaw_rate);

	} else if (setpoint_moves_towards_target_faster && !(rotating_towards_target && excess_yaw_rate > 0.0f)) {
		const float max_delta_yaw_rate = max_yaw_accel * dt_limited;
		yawspeed_sp = _yawspeed_sp_prev + math::constrain(yaw_rate_sp - _yawspeed_sp_prev,
				-max_delta_yaw_rate, max_delta_yaw_rate);

	} else {
		yawspeed_sp = yaw_rate_sp;
	}

	yawspeed_sp = math::constrain(yawspeed_sp, -max_yaw_rate, max_yaw_rate);
	_yawspeed_sp_prev = yawspeed_sp;
	_yawspeed_error_prev = yaw_rate_sp - yaw_rate;

	return yawspeed_sp;
}
