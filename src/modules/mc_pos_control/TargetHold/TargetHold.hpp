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
 * @file TargetHold.hpp
 *
 * Stay-above-target controller for the downward-facing IR beacon camera.
 *
 * Converts the camera's body-frame bearing angles into a tilt-compensated
 * NED ground offset (kills the pendulum-swing phantom motion of this
 * wire-suspended vehicle), then runs a PID on the offset to produce NED
 * velocity setpoints for the existing velocity control cascade.
 *
 * XY only - altitude is owned by the winch, heading by DF keep-heading.
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <mathlib/math/filter/AlphaFilter.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/ir_camera_report.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/target_hold_status.h>

using namespace time_literals;

class TargetHold : public ModuleParams
{
public:
	explicit TargetHold(ModuleParams *parent) : ModuleParams(parent) {}

	bool enabled() const { return _param_df_tgt_hold_en.get() != 0; }

	/**
	 * Run the target-hold controller.
	 *
	 * @param height_m height above ground (e.g. dist_bottom), must be valid
	 * @param height_valid rangefinder validity
	 * @param sticks_centered no manual override active
	 * @param dt loop time [s]
	 * @param vel_sp_ne output: NED north/east velocity setpoint [m/s]
	 * @return true when tracking is active and vel_sp_ne is valid
	 */
	bool update(float height_m, bool height_valid, bool sticks_centered, float dt,
		    matrix::Vector2f &vel_sp_ne);

	void reset();

private:
	bool computeOffset(float height_m, matrix::Vector2f &offset_ne);
	void publishStatus(bool active, bool target_valid, float height_m,
			   const matrix::Vector2f &offset_ne, const matrix::Vector2f &vel_sp_ne);

	static constexpr float OFFSET_LPF_CUTOFF = 2.f;      // Hz, on the NED offset
	static constexpr float DERIVATIVE_LPF_CUTOFF = 2.f;  // Hz, on the D term
	static constexpr float MIN_HEIGHT = 0.5f;            // m, projection clamp near ground
	static constexpr float MIN_DOWN_COMPONENT = 0.3f;    // reject line-of-sight not pointing down (cos ~72 deg)
	static constexpr hrt_abstime REPORT_TIMEOUT = 500_ms;

	uORB::Subscription _ir_camera_report_sub{ORB_ID(ir_camera_report)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Publication<target_hold_status_s> _status_pub{ORB_ID(target_hold_status)};

	ir_camera_report_s _report{};
	matrix::Quatf _q{};
	bool _have_attitude{false};

	bool _tracking{false};                   // currently engaged (for engage-radius hysteresis)
	AlphaFilter<matrix::Vector2f> _offset_lpf{};
	AlphaFilter<matrix::Vector2f> _derivative_lpf{};
	matrix::Vector2f _offset_prev{};
	bool _offset_prev_valid{false};
	matrix::Vector2f _integral{};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DF_TGT_HOLD_EN>) _param_df_tgt_hold_en,
		(ParamFloat<px4::params::DF_TGT_P>) _param_df_tgt_p,
		(ParamFloat<px4::params::DF_TGT_I>) _param_df_tgt_i,
		(ParamFloat<px4::params::DF_TGT_D>) _param_df_tgt_d,
		(ParamFloat<px4::params::DF_TGT_VMAX>) _param_df_tgt_vmax,
		(ParamFloat<px4::params::DF_TGT_ILIM>) _param_df_tgt_ilim,
		(ParamFloat<px4::params::DF_TGT_MAX_R>) _param_df_tgt_max_r,
		(ParamFloat<px4::params::DF_TGT_OFS_X>) _param_df_tgt_ofs_x,
		(ParamFloat<px4::params::DF_TGT_OFS_Y>) _param_df_tgt_ofs_y
	)
};
