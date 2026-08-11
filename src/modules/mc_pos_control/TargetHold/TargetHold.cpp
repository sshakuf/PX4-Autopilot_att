/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "TargetHold.hpp"

#include <mathlib/mathlib.h>

using namespace matrix;

bool TargetHold::computeOffset(const float height_m, Vector2f &offset_ne)
{
	// line-of-sight unit direction to the beacon in body frame
	// angle_x: + = target toward body +X (forward), angle_y: + = target toward +Y (right)
	Vector3f los_body(tanf(_report.angle_x), tanf(_report.angle_y), 1.f);
	los_body.normalize();

	// body -> NED with the full attitude: removes the pendulum-swing phantom
	// motion (a tilted camera otherwise reports the beacon shifted by the
	// tilt angle even when nothing moved)
	const Vector3f los_ned = _q.rotateVector(los_body);

	if (los_ned(2) < MIN_DOWN_COMPONENT) {
		// line of sight not pointing sufficiently down: bogus geometry
		return false;
	}

	const float h = math::max(height_m, MIN_HEIGHT);
	const float scale = h / los_ned(2);

	offset_ne(0) = los_ned(0) * scale;
	offset_ne(1) = los_ned(1) * scale;

	// camera lever arm: beacon centered in image means the CAMERA is above
	// the target; shift so the hold point puts the vehicle reference over it
	const Vector3f lever_body(_param_df_tgt_ofs_x.get(), _param_df_tgt_ofs_y.get(), 0.f);
	const Vector3f lever_ned = _q.rotateVector(lever_body);
	offset_ne(0) += lever_ned(0);
	offset_ne(1) += lever_ned(1);

	return offset_ne.isAllFinite();
}

bool TargetHold::update(const float height_m, const bool height_valid, const bool sticks_centered,
			const float dt, Vector2f &vel_sp_ne)
{
	_ir_camera_report_sub.update(&_report);

	vehicle_attitude_s att;

	if (_vehicle_attitude_sub.update(&att)) {
		_q = Quatf(att.q);
		_have_attitude = true;
	}

	const hrt_abstime now = hrt_absolute_time();

	const bool target_valid = _report.valid
				  && PX4_ISFINITE(_report.angle_x) && PX4_ISFINITE(_report.angle_y)
				  && (now - _report.timestamp < REPORT_TIMEOUT);

	Vector2f offset_ne{};
	bool offset_ok = false;

	if (target_valid && _have_attitude && height_valid) {
		offset_ok = computeOffset(height_m, offset_ne);
	}

	if (!enabled() || !offset_ok || !sticks_centered) {
		reset();
		publishStatus(false, target_valid, height_m, offset_ne, Vector2f{});
		return false;
	}

	// engage only within the capture radius (don't chase distant/false spots);
	// once tracking, keep following without a radius so we don't drop mid-correction
	if (!_tracking) {
		if (offset_ne.norm() > _param_df_tgt_max_r.get()) {
			publishStatus(false, target_valid, height_m, offset_ne, Vector2f{});
			return false;
		}

		_tracking = true;
		_offset_lpf.reset(offset_ne);
		_derivative_lpf.reset(Vector2f{});
		_offset_prev_valid = false;
		_integral.zero();
	}

	const float dt_c = math::constrain(dt, 0.002f, 0.1f);

	_offset_lpf.setParameters(dt_c, 1.f / (2.f * (float)M_PI * OFFSET_LPF_CUTOFF));
	_offset_lpf.update(offset_ne);
	const Vector2f offset_f = _offset_lpf.getState();

	// derivative on the filtered offset (DF_TGT_D, default 0)
	Vector2f derivative{};

	if (_offset_prev_valid) {
		derivative = (offset_f - _offset_prev) / dt_c;
	}

	_offset_prev = offset_f;
	_offset_prev_valid = true;

	_derivative_lpf.setParameters(dt_c, 1.f / (2.f * (float)M_PI * DERIVATIVE_LPF_CUTOFF));
	_derivative_lpf.update(derivative);

	const float vmax = math::max(_param_df_tgt_vmax.get(), 0.1f);

	Vector2f vel_cmd = offset_f * _param_df_tgt_p.get()
			   + _integral
			   + _derivative_lpf.getState() * _param_df_tgt_d.get();

	// integrate only when the command is not saturated (anti-windup)
	if (vel_cmd.norm() < vmax) {
		_integral += offset_f * _param_df_tgt_i.get() * dt_c;

		const float ilim = _param_df_tgt_ilim.get();

		if (_integral.norm() > ilim && ilim > FLT_EPSILON) {
			_integral = _integral.unit_or_zero() * ilim;
		}
	}

	if (vel_cmd.norm() > vmax) {
		vel_cmd = vel_cmd.unit_or_zero() * vmax;
	}

	vel_sp_ne = vel_cmd;

	publishStatus(true, target_valid, height_m, offset_f, vel_cmd);
	return true;
}

void TargetHold::reset()
{
	_tracking = false;
	_integral.zero();
	_offset_prev_valid = false;
}

void TargetHold::publishStatus(const bool active, const bool target_valid, const float height_m,
			       const Vector2f &offset_ne, const Vector2f &vel_sp_ne)
{
	target_hold_status_s status{};
	status.active = active;
	status.target_valid = target_valid;
	status.height = height_m;
	status.offset_n = offset_ne(0);
	status.offset_e = offset_ne(1);
	status.vel_cmd_n = vel_sp_ne(0);
	status.vel_cmd_e = vel_sp_ne(1);
	status.integral_n = _integral(0);
	status.integral_e = _integral(1);
	status.timestamp = hrt_absolute_time();
	_status_pub.publish(status);
}
