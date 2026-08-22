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
 * Horizontal thrust reserved for yaw
 *
 * Thrust magnitude held back from translation so the allocator always has
 * headroom to realise the yaw torque it is asked for.
 *
 * Several features write horizontal thrust independently - position/velocity
 * control, the swing damper (DF_SWAY_*), and the stick fallback - and their sum
 * was previously unbounded. Position control alone can reach MPC_THR_MAX while
 * the damper adds up to DF_SWAY_MAX on top, so the combined demand could exceed
 * full authority. The allocator then CLIPS per motor, which changes the
 * DIRECTION of the push rather than just its size, and once a motor is pinned at
 * zero the yaw differential cannot be produced at all.
 *
 * Measured (log_14_2000-01-01-00-15-32): a sustained 0.12 damper bias held motor
 * 1 at 0.000 for 92% of samples, actuator_saturation[1] mean -1.80, and
 * unallocated_torque[2] reached 0.079 - the vehicle drifted 45 deg off a heading
 * it had already reached, while the controller was asking for +22 deg/s and
 * could not get it. DF_YAW_PRIO_EN alone does not save this, because priority
 * re-allocation cannot recover a motor that is already at its limit.
 *
 * The horizontal budget is therefore
 *
 *     budget = MPC_THR_MAX - DF_YAW_RESERVE
 *
 * and when the summed demand exceeds it the total is SCALED, not clipped, so the
 * commanded direction is preserved and both contributions yield proportionally.
 * pos_control_health.thrust_scale reports the factor applied (1.0 = untouched)
 * and thrust_budget reports the limit, so starvation is visible in the log
 * instead of having to be inferred from motor saturation.
 *
 * 0 disables the reserve and restores the previous unbounded-sum behaviour
 * (translation may then starve yaw again).
 *
 * Raise it if unallocated_torque[2] is non-zero or yaw will not hold while
 * translating; lower it if translation feels weak and yaw has margin to spare.
 *
 * @min 0.0
 * @max 0.5
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(DF_YAW_RESERVE, 0.15f);
