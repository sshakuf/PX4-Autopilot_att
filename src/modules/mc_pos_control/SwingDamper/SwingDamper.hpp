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
 * @file SwingDamper.hpp
 *
 * DF_SWAY: pendulum swing damping from horizontal acceleration alone.
 *
 * The vehicle hangs on a long rope below a carrier drone, so it is a pendulum
 * with a MOVING suspension point. When the carrier stops, the payload keeps
 * swinging. This removes swing energy without needing any absolute position
 * reference - no optical flow, no GPS, no beacon.
 *
 * Why acceleration is a legitimate sensor here, unlike for position hold:
 * a swing is OSCILLATORY. Accelerometer bias and residual gravity leakage are
 * DC; the swing sits at the pendulum frequency (~0.16 Hz for a 10 m rope). A
 * band-pass separates them. Measured on this airframe (log_1_2026-8-17):
 *
 *     clutter in the 0.10-0.20 Hz band :  0.039 m/s^2
 *     DC..0.06 Hz drift floor          :  0.008-0.015 m/s^2
 *     swing signal at  2 deg amplitude :  0.342 m/s^2   (~9x SNR)
 *     swing signal at  5 deg amplitude :  0.856 m/s^2   (~22x SNR)
 *
 * Crucially this is NOT dead reckoning. The estimator is a LEAKY integrator,
 * so its output is bounded by construction:
 *
 *     v_dot = a - lambda * v          lambda = 2*pi*DF_SWAY_HP
 *
 * A constant acceleration bias produces a bounded v = a_bias / lambda (with
 * the defaults, 0.015 / 0.314 = 0.05 m/s), not the t^2 runaway that made the
 * flow-less position estimate diverge to 34 m of eph. At the swing frequency
 * (omega >> lambda) it behaves as a true integrator: ~96% amplitude with
 * ~18 deg of phase lag, which is harmless for damping (that tolerates +-90).
 *
 * Deliberately independent of the EKF horizontal solution: it consumes only
 * vehicle_local_position.ax/ay (IMU + attitude, published even when
 * xy_valid = 0) so it still works in exactly the flow-less condition it is
 * meant for.
 */

#pragma once

#include <matrix/matrix/math.hpp>

class SwingDamper
{
public:
	SwingDamper() = default;
	~SwingDamper() = default;

	/** DF_SWAY_EN. Disabling clears the filter state. */
	void setEnabled(bool enabled);

	/**
	 * @param damping    DF_SWAY_D    normalised thrust per m/s of swing velocity
	 * @param max_thrust DF_SWAY_MAX  hard clamp on the output magnitude
	 */
	void setGains(float damping, float max_thrust);

	/**
	 * @param washout_hz DF_SWAY_HP  leak frequency; must be well BELOW the swing
	 *                               frequency or the swing itself gets rejected
	 * @param lowpass_hz DF_SWAY_LP  anti-noise cutoff; must be well ABOVE it
	 */
	void setFilters(float washout_hz, float lowpass_hz);

	/**
	 * @param accel_ne NED horizontal acceleration, gravity already removed [m/s^2]
	 * @param dt       loop time [s]
	 * @return NED horizontal damping thrust to ADD to the existing setpoint,
	 *         normalised, magnitude <= DF_SWAY_MAX. Zero when disabled or when
	 *         the input is not finite.
	 */
	matrix::Vector2f update(const matrix::Vector2f &accel_ne, float dt);

	void reset();

	bool enabled() const { return _enabled; }
	/** band-passed swing velocity estimate [m/s], for logging */
	const matrix::Vector2f &velocity() const { return _velocity; }
	/** low-passed acceleration actually used [m/s^2], for logging */
	const matrix::Vector2f &accel() const { return _accel_lp; }

private:
	static constexpr float DT_MIN = 0.002f;
	static constexpr float DT_MAX = 0.1f;

	bool _enabled{false};
	float _damping{0.f};
	float _max_thrust{0.f};
	float _washout_hz{0.05f};
	float _lowpass_hz{0.5f};

	matrix::Vector2f _accel_lp{};   ///< low-passed acceleration
	matrix::Vector2f _velocity{};   ///< leaky-integrated (band-passed) velocity
};
