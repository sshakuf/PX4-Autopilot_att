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
 * @file IrCam.hpp
 *
 * Driver for the LSDT IR spot-detection camera (downward-facing beacon
 * tracker). Reads MSP V2 frames from a serial port, extracts the first
 * spot of every SpotsReport (function 298), converts the pixel position
 * to a center-relative offset and a body-frame bearing angle, and
 * publishes ir_camera_report.
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/ir_camera_report.h>
#include <uORB/topics/parameter_update.h>

/**
 * TEMPORARY BENCH DIAGNOSTIC -- remove when target-hold tuning is finished.
 *
 * Mirrors every ir_camera_report into debug_array, which the mavlink module
 * streams as DEBUG_FLOAT_ARRAY (50 Hz over USB in MAVLINK_MODE_CONFIG). That
 * makes the report readable by any ground-side MAVLink client -- pymavlink, or
 * MAVSDK via mavlink_direct -- because ir_camera_report is a custom uORB topic
 * that no MAVLink client can subscribe to directly.
 *
 * Set to 0 (or delete the guarded blocks in IrCam.hpp/IrCam.cpp) to remove.
 *
 * One debug_array is used rather than several DEBUG_VECTs on purpose:
 * MavlinkStreamDebugVect does a single _debug_sub.update() per tick and
 * debug_vect has queue depth 1, so back-to-back publications silently drop all
 * but the last. A single array publication has no such race.
 */
#define IR_CAM_DEBUG_MAVLINK 1

#if IR_CAM_DEBUG_MAVLINK
# include <uORB/topics/debug_array.h>
#endif

#include "MspV2Parser.hpp"

using namespace time_literals;

class IrCam : public px4::ScheduledWorkItem, public ModuleParams
{
public:
	IrCam(const char *port, unsigned baudrate);
	~IrCam() override;

	int init();
	void print_info();

	/** feed one byte through the parser (also used by the selftest) */
	void feedByte(uint8_t byte);

	static constexpr uint16_t FRAME_WIDTH = 1236;
	static constexpr uint16_t FRAME_HEIGHT = 960;

private:
	void Run() override;

	int openPort();
	void handleFrame(const MspV2Parser::Frame &frame);
	void publishReport(hrt_abstime now);
	void updateAngles();

	static constexpr uint16_t MSP_FUNC_SPOTS_REPORT = 298;
	static constexpr uint8_t SPOT_ENTRY_SIZE = 38;
	static constexpr uint8_t MAX_SPOTS = 6;

	static constexpr hrt_abstime STALE_TIMEOUT = 1_s;      // no valid spot for this long -> valid = false
	static constexpr hrt_abstime HEARTBEAT_INTERVAL = 100_ms; // publish at >= 10 Hz even without data

	char _port[32] {};
	unsigned _baudrate{921600};
	int _fd{-1};

	MspV2Parser _parser{};

	// last decoded valid spot (image-frame, center-relative)
	float _dx_px{0.f};
	float _dy_px{0.f};
	uint8_t _spot_id{0};
	uint32_t _spot_score{0};
	hrt_abstime _last_valid_spot_time{0};
	hrt_abstime _last_publish_time{0};

	uint32_t _spots_received{0};
	uint32_t _bytes_received{0};

	// frame-type statistics to localize "no target" problems
	uint32_t _spot_reports{0};        // SpotsReport (298) frames received
	uint32_t _spot_reports_empty{0};  // ... with spots_count == 0 (camera sees nothing)
	uint32_t _spots_not_valid{0};     // ... spot present but is_valid != 1
	uint32_t _logger_frames{0};       // Logger* diagnostics (310..316)
	uint32_t _other_frames{0};        // anything else
	uint16_t _last_other_function{0};

	uORB::Publication<ir_camera_report_s> _report_pub{ORB_ID(ir_camera_report)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

#if IR_CAM_DEBUG_MAVLINK
	// TEMPORARY BENCH DIAGNOSTIC -- see IR_CAM_DEBUG_MAVLINK above
	void publishDebugArray(const ir_camera_report_s &report);
	uORB::Publication<debug_array_s> _debug_array_pub{ORB_ID(debug_array)};
#endif

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _comms_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": comms errors")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DF_IRC_ROT>) _param_df_irc_rot,
		(ParamFloat<px4::params::DF_IRC_FX>) _param_df_irc_fx,
		(ParamFloat<px4::params::DF_IRC_FY>) _param_df_irc_fy,
		(ParamFloat<px4::params::DF_IRC_CX>) _param_df_irc_cx,
		(ParamFloat<px4::params::DF_IRC_CY>) _param_df_irc_cy,
		(ParamFloat<px4::params::DF_IRC_K1>) _param_df_irc_k1
	)
};
