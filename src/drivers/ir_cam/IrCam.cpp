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

#include "IrCam.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/log.h>

IrCam::IrCam(const char *port, unsigned baudrate) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	ModuleParams(nullptr),
	_baudrate(baudrate)
{
	strncpy(_port, port, sizeof(_port) - 1);
	_port[sizeof(_port) - 1] = '\0';
}

IrCam::~IrCam()
{
	ScheduleClear();

	if (_fd >= 0) {
		::close(_fd);
	}

	perf_free(_cycle_perf);
	perf_free(_comms_error_perf);
}

int IrCam::init()
{
	// publish an initial invalid report so consumers see the topic immediately
	publishReport(hrt_absolute_time());

	ScheduleOnInterval(5_ms);
	return PX4_OK;
}

int IrCam::openPort()
{
	_fd = ::open(_port, O_RDONLY | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		return -1;
	}

	struct termios config {};

	if (tcgetattr(_fd, &config) < 0) {
		::close(_fd);
		_fd = -1;
		return -1;
	}

	// raw 8N1
	cfmakeraw(&config);

	speed_t speed;

	switch (_baudrate) {
	case 57600:   speed = B57600;   break;

	case 115200:  speed = B115200;  break;

	case 230400:  speed = B230400;  break;

#ifdef B460800

	case 460800:  speed = B460800;  break;
#endif
#ifdef B921600

	case 921600:  speed = B921600;  break;
#endif

	default:
		PX4_ERR("unsupported baudrate %u", _baudrate);
		::close(_fd);
		_fd = -1;
		return -1;
	}

	cfsetispeed(&config, speed);
	cfsetospeed(&config, speed);

	if (tcsetattr(_fd, TCSANOW, &config) < 0) {
		::close(_fd);
		_fd = -1;
		return -1;
	}

	tcflush(_fd, TCIOFLUSH);

	PX4_INFO("opened %s @ %u", _port, _baudrate);
	return PX4_OK;
}

void IrCam::feedByte(uint8_t byte)
{
	MspV2Parser::Frame frame;

	if (_parser.parse(byte, frame)) {
		if (frame.crc_ok) {
			handleFrame(frame);
		}
	}
}

void IrCam::handleFrame(const MspV2Parser::Frame &frame)
{
	if (frame.function != MSP_FUNC_SPOTS_REPORT || frame.direction != '>') {
		// Logger*/other messages: not needed for target tracking
		if (frame.function >= 310 && frame.function <= 316) {
			_logger_frames++;

		} else {
			_other_frames++;
			_last_other_function = frame.function;
		}

		return;
	}

	if (frame.size < 1) {
		return;
	}

	_spot_reports++;

	uint8_t count = frame.payload[0];

	if (count == 0) {
		_spot_reports_empty++;
		return;
	}

	if (count > MAX_SPOTS) {
		return;
	}

	if (frame.size < (uint16_t)(1 + SPOT_ENTRY_SIZE)) {
		// no complete first spot entry
		return;
	}

	// only spots[0] is used (camera reports a single tracked target in practice)
	const uint8_t *p = frame.payload + 1;

	const uint8_t spot_id = p[0];
	const uint8_t is_valid = p[1];
	const uint16_t x = (uint16_t)p[2] | ((uint16_t)p[3] << 8);
	const uint16_t y = (uint16_t)p[4] | ((uint16_t)p[5] << 8);

	uint32_t score = 0;

	for (int i = 0; i < 4; i++) {
		// low 32 bits of the u64 score, enough for diagnostics
		score |= (uint32_t)p[23 + i] << (8 * i);
	}

	_spots_received++;

	if (is_valid == 1) {
		_dx_px = (float)x - (float)FRAME_WIDTH / 2.f;   // +right of center
		_dy_px = (float)y - (float)FRAME_HEIGHT / 2.f;  // +below center (image y-down)
		_spot_id = spot_id;
		_spot_score = score;
		_last_valid_spot_time = hrt_absolute_time();

		publishReport(_last_valid_spot_time);

	} else {
		_spots_not_valid++;
	}
}

void IrCam::publishReport(hrt_abstime now)
{
	const bool valid = (_last_valid_spot_time != 0) && (now - _last_valid_spot_time < STALE_TIMEOUT);

	ir_camera_report_s report{};
	report.frame_width = FRAME_WIDTH;
	report.frame_height = FRAME_HEIGHT;
	report.valid = valid;

	if (valid) {
		report.dx_px = _dx_px;
		report.dy_px = _dy_px;
		report.spot_id = _spot_id;
		report.spot_score = _spot_score;

	} else {
		// heartbeat with zeroed offsets: "no target", not "no driver"
		report.dx_px = 0.f;
		report.dy_px = 0.f;
	}

	report.angle_x = NAN;
	report.angle_y = NAN;

	const float fov_x_deg = _param_df_irc_fovx.get();
	const float fov_y_deg = _param_df_irc_fovy.get();

	if (valid && fov_x_deg > 1.f && fov_y_deg > 1.f) {
		// pinhole model: tan(angle) = dx / f, f = (width/2) / tan(fov/2)
		const float f_x = ((float)FRAME_WIDTH / 2.f) / tanf(math::radians(fov_x_deg) / 2.f);
		const float f_y = ((float)FRAME_HEIGHT / 2.f) / tanf(math::radians(fov_y_deg) / 2.f);

		const float angle_right_img = atan2f(report.dx_px, f_x);  // +right in image
		const float angle_down_img = atan2f(report.dy_px, f_y);   // +down in image (toward image bottom)

		// image -> body (camera looking down). DF_IRC_ROT = camera yaw mounting:
		// 0: image top = body +X (nose)  -> forward = -down_img, right = +right_img
		// 1: +90 deg                     -> forward = -right_img, right = -down_img
		// 2: 180 deg                     -> forward = +down_img,  right = -right_img
		// 3: 270 deg                     -> forward = +right_img, right = +down_img
		switch (_param_df_irc_rot.get()) {
		default:
		case 0:
			report.angle_x = -angle_down_img;
			report.angle_y = angle_right_img;
			break;

		case 1:
			report.angle_x = -angle_right_img;
			report.angle_y = -angle_down_img;
			break;

		case 2:
			report.angle_x = angle_down_img;
			report.angle_y = -angle_right_img;
			break;

		case 3:
			report.angle_x = angle_right_img;
			report.angle_y = angle_down_img;
			break;
		}
	}

	report.timestamp = hrt_absolute_time();
	_report_pub.publish(report);
	_last_publish_time = now;
}

void IrCam::Run()
{
	perf_begin(_cycle_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}

	if (_fd < 0) {
		if (openPort() != PX4_OK) {
			// port not available yet; retry on next cycles
			perf_count(_comms_error_perf);
			perf_end(_cycle_perf);
			return;
		}
	}

	// drain everything available this cycle
	uint8_t buf[256];

	for (int reads = 0; reads < 8; reads++) {
		int n = ::read(_fd, buf, sizeof(buf));

		if (n <= 0) {
			break;
		}

		_bytes_received += n;

		for (int i = 0; i < n; i++) {
			feedByte(buf[i]);
		}
	}

	// low-rate heartbeat so consumers always see a fresh timestamp,
	// and valid drops to false on staleness
	const hrt_abstime now = hrt_absolute_time();

	if (now - _last_publish_time >= HEARTBEAT_INTERVAL) {
		publishReport(now);
	}

	perf_end(_cycle_perf);
}

void IrCam::print_info()
{
	PX4_INFO("port: %s @ %u baud (fd %d)", _port, _baudrate, _fd);
	PX4_INFO("bytes: %lu, frames ok: %lu, crc errors: %lu, spots: %lu",
		 (unsigned long)_bytes_received,
		 (unsigned long)_parser.framesOk(),
		 (unsigned long)_parser.crcErrors(),
		 (unsigned long)_spots_received);
	PX4_INFO("frames by type: spot reports: %lu (empty: %lu, not-valid spots: %lu), logger: %lu, other: %lu (last func %u)",
		 (unsigned long)_spot_reports,
		 (unsigned long)_spot_reports_empty,
		 (unsigned long)_spots_not_valid,
		 (unsigned long)_logger_frames,
		 (unsigned long)_other_frames,
		 _last_other_function);

	const hrt_abstime now = hrt_absolute_time();

	if (_last_valid_spot_time != 0) {
		PX4_INFO("last valid spot: %.3f s ago, dx=%.1f px dy=%.1f px (id %u, score %lu)",
			 (double)((now - _last_valid_spot_time) / 1e6),
			 (double)_dx_px, (double)_dy_px, _spot_id, (unsigned long)_spot_score);

	} else {
		PX4_INFO("no valid spot received yet");
	}

	perf_print_counter(_cycle_perf);
	perf_print_counter(_comms_error_perf);
}
