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
 * Invalid-input thrust fade-out time
 *
 * How long the horizontal thrust setpoint is faded to zero when the position
 * controller loses a usable input, instead of being frozen at its last value.
 *
 * The horizontal loop only runs while PositionControl::_inputValid() passes. It
 * needs finite velocity AND velocity-derivative estimates, so removing the
 * optical flow sets v_xy_valid = 0, which NANs states.velocity.xy() and makes
 * the check fail on every iteration. Without this fade the thrust setpoint kept
 * its last value indefinitely: a constant 0.392 push held in a fixed compass
 * direction while the airframe rotated underneath it, which looked like the
 * vehicle flying off in random directions (log_0_2026-8-17-15-40-32).
 *
 * Fading rather than zeroing outright means a brief dropout passes through
 * without a step discontinuity in thrust, while a sustained loss still decays to
 * zero in a bounded time.
 *
 * Set to 0 to zero the thrust immediately on loss of validity. Larger values
 * ride out longer dropouts but keep commanding a stale direction for longer -
 * and the direction is stale in EARTH frame, so it becomes progressively more
 * wrong as the vehicle yaws.
 *
 * Note this fades only the position/velocity contribution. The swing damper
 * (DF_SWAY_EN) is independent of the estimator and keeps running, so with a
 * fully faded setpoint the vehicle is left under pure swing damping.
 *
 * @min 0.0
 * @max 5.0
 * @unit s
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(DF_INVALID_DECAY, 0.5f);
