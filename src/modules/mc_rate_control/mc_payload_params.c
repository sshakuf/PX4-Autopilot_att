/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * Direct Flight: Current payload mass
 *
 * Scales all three rate-loop PID gains based on payload (roll, pitch, yaw).
 * Rate gains (MC_ROLLRATE_*, MC_PITCHRATE_*, MC_YAWRATE_*) are assumed tuned
 * at full payload (45 kg). Lighter payloads reduce inertia on every axis,
 * so gains are scaled down linearly to avoid rate-loop instability.
 *
 * Scale: payload_min + (1 - payload_min) * (DF_PAYLOAD_KG / 45)
 * At 45 kg: scale = 1.0 (no change). At 0 kg: scale = DF_PAYLOAD_MIN.
 *
 * @unit kg
 * @min 0.0
 * @max 45.0
 * @decimal 1
 * @increment 1.0
 * @reboot_required false
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(DF_PAYLOAD_KG, 45.0f);

/**
 * Direct Flight: Minimum rate-gain scale at empty payload
 *
 * Floor of the linear scaling applied by DF_PAYLOAD_KG. When payload = 0,
 * roll/pitch rate gains are multiplied by this value. Smaller = softer
 * response on empty drone.
 *
 * Typical: 0.25 if drone-only mass is ~15 kg vs 45 kg full payload.
 *
 * @min 0.001
 * @max 2.0
 * @decimal 2
 * @increment 0.05
 * @reboot_required false
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(DF_PAYLOAD_MIN, 0.25f);
