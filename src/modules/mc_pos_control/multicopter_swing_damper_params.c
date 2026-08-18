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
 * Swing damper: enable
 *
 * Damps the pendulum swing of a payload hanging on a rope below a carrier
 * drone. When the carrier stops, the payload keeps swinging; this adds a
 * horizontal thrust opposing the swing velocity.
 *
 * Uses acceleration only - no optical flow, GPS or IR beacon. It works while
 * the EKF horizontal solution is invalid (xy_valid = 0), because the input is
 * IMU + attitude derived. It is also immune to heading error, since the
 * estimate and the command are rotated by the same yaw.
 *
 * It REDUCES swing, it does not hold position. While the carrier keeps moving
 * the swing is continuously re-driven and can only be damped, not nulled.
 *
 * Measured on this airframe: swing at 0.42 Hz (period 2.4 s, effective
 * pendulum 1.4 m), disturbance 1.6-3.4 m/s^2. Damping 83-90% of swing
 * amplitude depending on gain and limit.
 *
 * Tuning order: DF_SWAY_LP and DF_SWAY_HP first (they must bracket the swing
 * frequency), then DF_SWAY_MAX for authority, then DF_SWAY_D for speed.
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(DF_SWAY_EN, 0);

/**
 * Swing damper: thrust limit (authority)
 *
 * Hard clamp on the magnitude of the damping thrust. This is BOTH the safety
 * bound and, for a continuously re-driven swing, the main thing that decides
 * how well the damper works.
 *
 * Available damping acceleration = DF_SWAY_MAX * DF_ACC_PER_THR. If that is
 * below the disturbance the damper is out-gunned and gain will not rescue it:
 *
 *   DF_SWAY_MAX 0.30 -> 1.2 m/s^2 : lost to a 1.6-3.4 m/s^2 disturbance
 *   DF_SWAY_MAX 0.50 -> 2.0 m/s^2 : 88% swing reduction
 *   DF_SWAY_MAX 0.65 -> 2.6 m/s^2 : 90%, but leaves nothing for anything else
 *
 * Keep it at or below MPC_THR_MAX. Raise this before raising DF_SWAY_D: with
 * too little authority, extra gain only spends more time saturated (69% of
 * samples at MAX 0.30 / D 0.35) without damping any better.
 *
 * @min 0.0
 * @max 0.9
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(DF_SWAY_MAX, 0.50f);

/**
 * Swing damper: gain
 *
 * Normalised horizontal thrust per m/s of estimated swing velocity.
 *
 * For a free (undriven) swing the amplitude decay time constant is about
 * 0.5 / DF_SWAY_D seconds, independent of rope length:
 *
 *   0.15 -> 3.3 s      0.35 -> 1.4 s      0.50 -> 1.0 s
 *
 * Raise DF_SWAY_MAX first - gain beyond the available authority just saturates.
 *
 * CAUTION: the damping force reacts back up the rope onto the carrier drone.
 * With a 10 m line this is a coupled two-body system, so an aggressive value
 * here can fight the carrier's own position hold. Increase in steps and watch
 * whether the carrier starts working against you.
 *
 * @min 0.0
 * @max 1.5
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(DF_SWAY_D, 0.35f);

/**
 * Swing damper: washout frequency
 *
 * Leak frequency of the velocity estimator. This is what makes the estimator
 * safe: it is a leaky integrator, not dead reckoning, so an acceleration bias
 * settles to a bounded offset instead of diverging.
 *
 * Must sit well BELOW the swing frequency, or the swing is rejected with the
 * bias. Pendulum frequency is f = sqrt(g/L)/(2*pi):
 *
 *   L = 1.4 m -> 0.42 Hz (measured here)    L = 10 m -> 0.16 Hz
 *
 * Higher values waste less gain on bias. Phantom velocity from a bias is
 * a_bias / (2*pi*DF_SWAY_HP), so with the 0.4 m/s^2 EKF2_ABL_LIM worst case:
 *
 *   0.05 Hz -> 1.27 m/s wasted     0.20 Hz -> 0.32 m/s wasted
 *   0.10 Hz -> 0.64 m/s wasted     0.30 Hz -> 0.21 m/s wasted
 *
 * It also sets the settling time, about 3/(2*pi*DF_SWAY_HP) seconds - roughly
 * 2.5 s at the default. Do not judge the damper before then.
 *
 * @min 0.005
 * @max 1.0
 * @unit Hz
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(DF_SWAY_HP, 0.20f);

/**
 * Swing damper: low-pass frequency
 *
 * Anti-vibration cutoff on the acceleration input. Must sit well ABOVE the
 * swing frequency so the swing passes untouched, while keeping motor and frame
 * vibration out of the integrator.
 *
 * Setting this too close to the swing frequency both attenuates the signal and
 * adds phase lag: at 0.5 Hz against the measured 0.42 Hz swing it cost about
 * 40 deg of phase. Keep it at least 4x the swing frequency.
 *
 * @min 0.05
 * @max 10.0
 * @unit Hz
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(DF_SWAY_LP, 2.0f);
