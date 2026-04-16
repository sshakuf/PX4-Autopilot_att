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
 * Enable keep heading feature
 *
 * When enabled, the position controller will maintain the heading
 * specified by DF_YAW_HOLD parameter instead of following
 * trajectory setpoint yaw commands.
 *
 * @min 0
 * @max 1
 * @value 0 Disabled
 * @value 1 Enabled
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(DF_YAW_HOLD_EN, 0);

/**
 * Keep heading target angle
 *
 * Target heading in degrees using NED (North-East-Down) coordinates.
 * 0° = True North, 90° = East, 180° = South, 270° = West.
 * This heading will be maintained when DF_YAW_HOLD_EN is enabled.
 *
 * @unit deg
 * @min 0.0
 * @max 360.0
 * @decimal 1
 * @increment 1.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(DF_YAW_HOLD, 0.0f);

/**
 * Maximum desired yaw rate for keep heading
 *
 * Maximum yaw rate setpoint when keep heading is enabled.
 * This limits the commanded rotation speed during heading corrections.
 * Higher values allow faster heading corrections but may cause oscillations.
 * Lower values provide smoother rotation but slower heading acquisition.
 *
 * @unit deg/s
 * @min 10.0
 * @max 180.0
 * @decimal 1
 * @increment 5.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(DF_YAWSPEED_MAXR, 90.0f);

/**
 * Yaw speed proportional gain
 *
 * Proportional gain for yaw speed control when keep heading is enabled.
 * Higher values result in faster response to yaw rate errors.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(DF_YAWSPEED_P, 1.0f);

/**
 * Yaw speed integral gain
 *
 * Integral gain for yaw speed control when keep heading is enabled.
 * Eliminates steady-state errors in yaw rate tracking.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(DF_YAWSPEED_I, 0.1f);

/**
 * Yaw speed derivative gain
 *
 * Derivative gain for yaw speed control when keep heading is enabled.
 * Dampens oscillations and improves stability.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(DF_YAWSPEED_D, 0.05f);
