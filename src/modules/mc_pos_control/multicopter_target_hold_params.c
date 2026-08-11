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
 * Enable IR-beacon stay-above-target mode
 *
 * When enabled and the IR camera reports a valid beacon (ir_camera_report,
 * requires DF_IRC_FOVX/FOVY calibrated), the position controller commands
 * velocity setpoints that keep the vehicle above the beacon. Engages only
 * when the sticks are centered and the beacon is within DF_TGT_MAX_R.
 * Sticks always override; on release the beacon is re-captured. XY only:
 * altitude and heading are not touched.
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(DF_TGT_HOLD_EN, 0);

/**
 * Target hold proportional gain
 *
 * Velocity commanded per meter of beacon offset.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(DF_TGT_P, 0.5f);

/**
 * Target hold integral gain
 *
 * Trims steady-state standoff caused by constant disturbances
 * (rope tension bias, wind). Integrator authority is limited by
 * DF_TGT_ILIM and reset on disengage/stick input/target loss.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(DF_TGT_I, 0.05f);

/**
 * Target hold derivative gain
 *
 * Damping on the beacon offset rate. The inner velocity loop already
 * provides damping via optical flow, so this defaults to 0 - available
 * for tuning if extra damping on the camera signal is needed.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(DF_TGT_D, 0.0f);

/**
 * Target hold maximum commanded velocity
 *
 * @unit m/s
 * @min 0.1
 * @max 3.0
 * @decimal 1
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(DF_TGT_VMAX, 0.5f);

/**
 * Target hold integrator limit
 *
 * Maximum velocity contribution of the integral term.
 *
 * @unit m/s
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(DF_TGT_ILIM, 0.3f);

/**
 * Target hold capture radius
 *
 * Beacon offsets larger than this are not engaged (prevents chasing
 * distant or false detections). Once tracking, the radius no longer
 * applies so a correction in progress is not dropped.
 *
 * @unit m
 * @min 0.5
 * @max 10.0
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(DF_TGT_MAX_R, 3.0f);

/**
 * IR camera position offset, body X
 *
 * Camera mounting position forward of the vehicle reference point.
 * Used so "beacon centered" means the vehicle - not the camera - is
 * above the target.
 *
 * @unit m
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(DF_TGT_OFS_X, 0.0f);

/**
 * IR camera position offset, body Y
 *
 * Camera mounting position right of the vehicle reference point.
 *
 * @unit m
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(DF_TGT_OFS_Y, 0.0f);
