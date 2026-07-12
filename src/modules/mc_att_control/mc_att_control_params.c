/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_params.c
 * Parameters for multicopter attitude controller.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

/**
 * Roll P gain
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLL_P, 4.0f);

/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCH_P, 4.0f);

/**
 * Yaw P gain
 *
 * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @max 5
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_P, 2.8f);

/**
 * Yaw weight
 *
 * A fraction [0,1] deprioritizing yaw compared to roll and pitch in non-linear attitude control.
 * Deprioritizing yaw is necessary because multicopters have much less control authority
 * in yaw compared to the other axes and it makes sense because yaw is not critical for
 * stable hovering or 3D navigation.
 *
 * For yaw control tuning use MC_YAW_P. This ratio has no impact on the yaw gain.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_WEIGHT, 0.4f);

/**
 * Max roll rate
 *
 * Limit for roll rate in manual and auto modes (except acro).
 * Has effect for large rotations in autonomous mode, to avoid large control
 * output and mixer saturation.
 *
 * This is not only limited by the vehicle's properties, but also by the maximum
 * measurement rate of the gyro.
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_MAX, 220.0f);

/**
 * Max pitch rate
 *
 * Limit for pitch rate in manual and auto modes (except acro).
 * Has effect for large rotations in autonomous mode, to avoid large control
 * output and mixer saturation.
 *
 * This is not only limited by the vehicle's properties, but also by the maximum
 * measurement rate of the gyro.
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_MAX, 220.0f);

/**
 * Max yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_MAX, 200.0f);

/**
 * Manual tilt input filter time constant
 *
 * Setting this parameter to 0 disables the filter
 *
 * @unit s
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_MAN_TILT_TAU, 0.0f);

/**
 * Direct Flight: Enable direct RC to thrust/torque control
 *
 * Bypasses attitude and rate controllers in manual modes.
 * RC stick inputs directly command thrust and torque to motors.
 * Works in Position, Altitude, and Acro modes when enabled.
 *
 * Designed for constrained vehicles (e.g., horizontal drones on tether)
 * where direct control is preferred over stabilization.
 *
 * @boolean
 * @reboot_required false
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_INT32(DF_MC_DIR_EN, 0);

/**
 * Direct Flight: Roll/Pitch scaling factor
 *
 * Scales RC roll/pitch stick inputs to roll/pitch TORQUE (not thrust).
 * Uses same control allocation as yaw for full motor response. 1.0 = 1:1.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(DF_MC_DIR_RP, 1.0f);

/**
 * Direct Flight: Yaw scaling factor
 *
 * Scales RC yaw stick input [-1,1] to yaw torque output.
 * 1.0 = true 1:1 (stick 100% → torque 100%). Params refresh at runtime.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(DF_MC_DIR_YAW, 1.0f);

/**
 * Direct Flight: Thrust scaling factor
 *
 * Scales RC throttle stick input to thrust output.
 * For horizontal drones, may need adjustment based on configuration.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(DF_MC_DIR_THR, 1.0f);

/**
 * Enable keep heading in attitude (stabilized) mode
 *
 * When enabled and flying in manual/stabilized mode, the attitude
 * controller holds the heading given by DF_YAW_HOLD using the same
 * keep-heading yaw-rate PID (DF_YAW*) as position mode. Roll/pitch
 * sticks command torque directly (scaled by DF_MC_DIR_RP) and the
 * throttle stick commands thrust (scaled by DF_MC_DIR_THR); the yaw
 * stick is ignored. Takes precedence over DF_MC_DIR_EN.
 *
 * @boolean
 * @reboot_required false
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_INT32(DF_ATT_HOLD_EN, 0);
