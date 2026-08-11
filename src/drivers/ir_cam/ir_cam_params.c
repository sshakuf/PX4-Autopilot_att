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
 * IR camera mounting rotation
 *
 * Yaw orientation of the downward-facing IR camera relative to the vehicle
 * body, in 90 degree steps. Determines how the image-frame target offset
 * maps to body-frame bearing angles:
 * 0 = image top points to vehicle nose (+X)
 * 1 = image top points to vehicle right (+Y)
 * 2 = image top points to vehicle tail (-X)
 * 3 = image top points to vehicle left (-Y)
 *
 * Verify on the bench: place the beacon toward the vehicle nose; angle_x in
 * ir_camera_report must be positive.
 *
 * @min 0
 * @max 3
 * @value 0 Image top = nose
 * @value 1 Image top = right
 * @value 2 Image top = tail
 * @value 3 Image top = left
 * @group Sensors
 */
PARAM_DEFINE_INT32(DF_IRC_ROT, 0);

/**
 * IR camera horizontal field of view
 *
 * Angular field of view across the image width (1236 px). Required for
 * converting the pixel offset into a bearing angle; 0 disables the angle
 * output (ir_camera_report.angle_x/angle_y = NAN, pixel offsets still
 * published). Calibrate: place the beacon at a known lateral distance d and
 * known height h below the camera, read dx_px, then
 * FOV = 2*atan( (width/2) * tan(atan(d/h)) / dx_px ).
 *
 * @unit deg
 * @min 0.0
 * @max 180.0
 * @decimal 1
 * @group Sensors
 */
PARAM_DEFINE_FLOAT(DF_IRC_FOVX, 0.0f);

/**
 * IR camera vertical field of view
 *
 * Angular field of view across the image height (960 px). See DF_IRC_FOVX.
 *
 * @unit deg
 * @min 0.0
 * @max 180.0
 * @decimal 1
 * @group Sensors
 */
PARAM_DEFINE_FLOAT(DF_IRC_FOVY, 0.0f);
