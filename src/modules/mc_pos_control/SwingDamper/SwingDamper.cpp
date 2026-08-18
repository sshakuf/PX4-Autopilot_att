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

/**
 * @file SwingDamper.cpp
 */

#include "SwingDamper.hpp"

#include <mathlib/mathlib.h>

using namespace matrix;

void SwingDamper::setEnabled(bool enabled)
{
	if (enabled != _enabled) {
		// Clear on both edges. On enable the filters must not start with stale
		// state from a previous run; on disable nothing should linger.
		reset();
	}

	_enabled = enabled;
}

void SwingDamper::setGains(float damping, float max_thrust)
{
	_damping = math::max(damping, 0.f);
	_max_thrust = math::max(max_thrust, 0.f);
}

void SwingDamper::setFilters(float washout_hz, float lowpass_hz)
{
	// Guard rails rather than blind acceptance: a washout at 0 would turn the
	// leaky integrator into a pure integrator and reintroduce the unbounded
	// bias ramp this design exists to avoid.
	_washout_hz = math::constrain(washout_hz, 0.005f, 1.f);
	_lowpass_hz = math::constrain(lowpass_hz, 0.05f, 10.f);
}

void SwingDamper::reset()
{
	_accel_lp.setZero();
	_velocity.setZero();
}

Vector2f SwingDamper::update(const Vector2f &accel_ne, float dt)
{
	if (!_enabled) {
		reset();
		return Vector2f{};
	}

	if (!accel_ne.isAllFinite() || !PX4_ISFINITE(dt)) {
		// Never propagate a NAN into the thrust setpoint. Hold the current
		// estimate rather than resetting, so a single bad sample does not throw
		// away a converged filter.
		return Vector2f{};
	}

	const float dt_c = math::constrain(dt, DT_MIN, DT_MAX);

	// 1) low-pass the acceleration: rejects motor//frame vibration far above the
	//    pendulum frequency, which integration would otherwise accumulate.
	const float tau_lp = 1.f / (2.f * M_PI_F * _lowpass_hz);
	const float alpha = dt_c / (tau_lp + dt_c);
	_accel_lp += (accel_ne - _accel_lp) * alpha;

	// 2) leaky integrator: integrator in the swing band, high-pass below it.
	//        v_dot = a - lambda * v
	//    Bounded for any constant bias (v -> a_bias / lambda), so this cannot
	//    run away the way double-integrated dead reckoning does.
	const float lambda = 2.f * M_PI_F * _washout_hz;
	_velocity += (_accel_lp - _velocity * lambda) * dt_c;

	if (!_velocity.isAllFinite()) {
		reset();
		return Vector2f{};
	}

	// 3) oppose the swing velocity, hard-clamped. Clamping the magnitude rather
	//    than each axis keeps the direction of the correction intact.
	Vector2f thrust = -_velocity * _damping;
	const float norm = thrust.norm();

	if (norm > _max_thrust) {
		thrust = (norm > FLT_EPSILON) ? thrust * (_max_thrust / norm) : Vector2f{};
	}

	return thrust;
}
