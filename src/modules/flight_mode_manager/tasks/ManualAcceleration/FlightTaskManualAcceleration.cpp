/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskManualAcceleration.cpp
 */

#include "FlightTaskManualAcceleration.hpp"
#include <px4_platform_common/log.h>

using namespace matrix;

bool FlightTaskManualAcceleration::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();

	_sticks.checkAndUpdateStickInputs();

	// [DBG_POS] FlightTask: rate-limited debug every ~1s
	// static int dbg_cnt = 0;
	// const bool dbg = (++dbg_cnt % 50) == 0;

	// if (dbg) {
	// 	PX4_INFO("[DBG_POS] FT_Accel updateInit: ret=%d relax=%d sticks_avail=%d z=%d vz=%d yaw=%d",
	// 		 ret, (int)_param_df_pos_relax.get(), _sticks.isAvailable(),
	// 		 PX4_ISFINITE(_position(2)), PX4_ISFINITE(_velocity(2)), PX4_ISFINITE(_yaw));
	// }

	// DF_POS_RELAX: allow running without valid z, vz, yaw, or sticks (horizontal tethered drone)
	// Parent requires these for standard operation; relax when positioning is not applicable
	if (_param_df_pos_relax.get() == 1) {
		// if (dbg) { PX4_INFO("[DBG_POS] FT_Accel: DF_POS_RELAX=1, returning %d", ret); }
		return ret;
	}

	// Standard checks from FlightTaskManualAltitude
	if (_sticks_data_required) {
		ret = ret && _sticks.isAvailable();
	}

	const bool out = ret && PX4_ISFINITE(_position(2)) && PX4_ISFINITE(_velocity(2)) && PX4_ISFINITE(_yaw);
	// if (dbg && !out) {
	// 	PX4_WARN("[DBG_POS] FT_Accel updateInit FAIL: ret=%d z=%d vz=%d yaw=%d",
	// 		 ret, PX4_ISFINITE(_position(2)), PX4_ISFINITE(_velocity(2)), PX4_ISFINITE(_yaw));
	// }
	return out;
}

bool FlightTaskManualAcceleration::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTaskManualAltitudeSmoothVel::activate(last_setpoint);

	_stick_acceleration_xy.resetPosition();

	if (Vector2f(last_setpoint.velocity).isAllFinite()) {
		_stick_acceleration_xy.resetVelocity(Vector2f(last_setpoint.velocity));

	} else {
		_stick_acceleration_xy.resetVelocity(_velocity.xy());
	}

	_stick_acceleration_xy.resetAcceleration(Vector2f(last_setpoint.acceleration));

	return ret;
}

bool FlightTaskManualAcceleration::update()
{
	const vehicle_local_position_s vehicle_local_pos = _sub_vehicle_local_position.get();
	setMaxDistanceToGround(vehicle_local_pos.hagl_max_xy);
	bool ret = FlightTaskManualAltitudeSmoothVel::update();

	float max_hagl_ratio = 0.0f;

	if (PX4_ISFINITE(vehicle_local_pos.hagl_max_xy) && vehicle_local_pos.hagl_max_xy > FLT_EPSILON) {
		max_hagl_ratio = (vehicle_local_pos.dist_bottom) / vehicle_local_pos.hagl_max_xy;
	}

	// limit horizontal velocity near max hagl to decrease chance of larger gound distance jumps
	static constexpr float factor_threshold = 0.8f; // threshold ratio of max_hagl
	static constexpr float min_vel = 2.f; // minimum max-velocity near max_hagl

	if (max_hagl_ratio > factor_threshold) {
		const float vxy_max = math::min(vehicle_local_pos.vxy_max, _param_mpc_vel_manual.get());
		_stick_acceleration_xy.setVelocityConstraint(interpolate(max_hagl_ratio, factor_threshold, 1.f, vxy_max, min_vel));

	} else if (PX4_ISFINITE(vehicle_local_pos.vxy_max)) {
		_stick_acceleration_xy.setVelocityConstraint(vehicle_local_pos.vxy_max);
	}

	// Use yaw fallback when invalid (DF_POS_RELAX: heading not stable)
	const float yaw = PX4_ISFINITE(_yaw) ? _yaw : (PX4_ISFINITE(_yaw_setpoint) ? _yaw_setpoint : 0.f);
	_stick_acceleration_xy.generateSetpoints(_sticks.getPitchRollExpo(), yaw, _yaw_setpoint, _position,
			_velocity_setpoint_feedback.xy(), _deltatime);
	_stick_acceleration_xy.getSetpoints(_position_setpoint, _velocity_setpoint, _acceleration_setpoint);

	// [DBG_POS] FlightTask: stick-derived setpoints
	// static int dbg_sp_cnt = 0;
	// if ((++dbg_sp_cnt % 50) == 0) {
	// 	Vector2f stick_pr = _sticks.getPitchRollExpo();
	// 	PX4_INFO("[DBG_POS] FT_Accel SP: stick[%.2f,%.2f] acc[%.3f,%.3f] vel[%.2f,%.2f] pos_sp[%.1f,%.1f]",
	// 		 (double)stick_pr(0), (double)stick_pr(1),
	// 		 (double)_acceleration_setpoint(0), (double)_acceleration_setpoint(1),
	// 		 (double)_velocity_setpoint(0), (double)_velocity_setpoint(1),
	// 		 (double)_position_setpoint(0), (double)_position_setpoint(1));
	// }

	_constraints.want_takeoff = _checkTakeoff();

	// check if an external yaw handler is active and if yes, let it update the yaw setpoints
	_weathervane.update();

	if (_weathervane.isActive()) {
		_yaw_setpoint = NAN;

		// only enable the weathervane to change the yawrate when position lock is active (and thus the pos. sp. are NAN)
		if (Vector2f(_position_setpoint).isAllFinite()) {
			// vehicle is steady
			_yawspeed_setpoint += _weathervane.getWeathervaneYawrate();
		}
	}

	return ret;
}

void FlightTaskManualAcceleration::_ekfResetHandlerPositionXY(const matrix::Vector2f &delta_xy)
{
	_stick_acceleration_xy.addToPositionSetpoint(delta_xy);
}

void FlightTaskManualAcceleration::_ekfResetHandlerVelocityXY(const matrix::Vector2f &delta_vxy)
{
	_stick_acceleration_xy.resetVelocity(_velocity.xy());
}
