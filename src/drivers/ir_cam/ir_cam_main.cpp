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

#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <string.h>

static IrCam *g_dev{nullptr};

static int selftest()
{
	int failures = 0;

	// 1. CRC-8/DVB-S2 standard check value
	const uint8_t check[] = "123456789";
	const uint8_t crc = crc8_dvb_s2(check, 9, 0);

	if (crc != 0xBC) {
		PX4_ERR("CRC selftest FAILED: got 0x%02X, expected 0xBC", crc);
		failures++;

	} else {
		PX4_INFO("CRC selftest OK (0xBC)");
	}

	// 2. Canonical SpotsReport frame from the validated project spec:
	//    one valid spot at (x=437, y=261), spot_id=2
	const uint8_t canonical[] = {
		0x24, 0x58, 0x3e, 0x00, 0x2a, 0x01, 0x27, 0x00, 0x01, 0x02, 0x01, 0xb5,
		0x01, 0x05, 0x01, 0x4b, 0x4e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x7f, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16
	};

	// prepend garbage to prove resync, feed byte-at-a-time
	const uint8_t garbage[] = {0x00, 0xff, 0x24, 0x99, 0x58};

	MspV2Parser parser;
	MspV2Parser::Frame frame{};
	bool got_frame = false;

	for (uint8_t b : garbage) {
		if (parser.parse(b, frame)) {
			got_frame = true;
		}
	}

	if (got_frame) {
		PX4_ERR("resync selftest FAILED: frame emitted from garbage");
		failures++;
	}

	for (uint8_t b : canonical) {
		if (parser.parse(b, frame)) {
			got_frame = true;
		}
	}

	if (!got_frame || !frame.crc_ok || frame.function != 298 || frame.size != 39) {
		PX4_ERR("frame selftest FAILED: got=%d crc_ok=%d func=%u size=%u",
			got_frame, frame.crc_ok, frame.function, frame.size);
		failures++;

	} else {
		const uint8_t *p = frame.payload;
		const uint16_t x = (uint16_t)p[3] | ((uint16_t)p[4] << 8);
		const uint16_t y = (uint16_t)p[5] | ((uint16_t)p[6] << 8);

		if (p[0] != 1 || p[1] != 2 || p[2] != 1 || x != 437 || y != 261) {
			PX4_ERR("payload selftest FAILED: count=%u id=%u valid=%u x=%u y=%u", p[0], p[1], p[2], x, y);
			failures++;

		} else {
			PX4_INFO("frame selftest OK (spot id 2 at 437,261)");
		}
	}

	// 3. corruption: flip one payload byte, CRC must fail
	uint8_t corrupted[sizeof(canonical)];
	memcpy(corrupted, canonical, sizeof(canonical));
	corrupted[11] ^= 0x01; // x low byte

	MspV2Parser parser2;
	bool got2 = false;
	MspV2Parser::Frame frame2{};

	for (uint8_t b : corrupted) {
		if (parser2.parse(b, frame2)) {
			got2 = true;
		}
	}

	if (!got2 || frame2.crc_ok) {
		PX4_ERR("corruption selftest FAILED: got=%d crc_ok=%d", got2, frame2.crc_ok);
		failures++;

	} else {
		PX4_INFO("corruption selftest OK (bad CRC rejected)");
	}

	PX4_INFO("selftest: %s", failures == 0 ? "ALL PASSED" : "FAILURES");
	return failures == 0 ? PX4_OK : PX4_ERROR;
}

static int print_usage(const char *reason = nullptr)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial driver for the LSDT IR spot-detection camera (downward-facing beacon
tracker). Parses the MSP V2 telemetry stream (SpotsReport, function 298) and
publishes ir_camera_report with the target pixel offset from frame center and,
when DF_IRC_FOVX/DF_IRC_FOVY are configured, the body-frame bearing angles.

Receive-only: nothing is ever transmitted to the camera.

The serial port is configured with the DF_IRC_CFG parameter; set the port's
SER_xxx_BAUD to 921600 (the camera's fixed rate).
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ir_cam", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('b', 921600, 57600, 921600, "Baudrate", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("selftest", "Run the parser against the known-good spec test vectors");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Driver status and link statistics");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop the driver");

	return PX4_ERROR;
}

extern "C" __EXPORT int ir_cam_main(int argc, char *argv[])
{
	if (argc < 2) {
		return print_usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {
		if (g_dev != nullptr) {
			PX4_WARN("already started");
			return PX4_OK;
		}

		const char *device = nullptr;
		int baudrate = 921600;

		int myoptind = 1;
		int ch;
		const char *myoptarg = nullptr;

		while ((ch = px4_getopt(argc, argv, "d:b:", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'd':
				device = myoptarg;
				break;

			case 'b':
				if (px4_get_parameter_value(myoptarg, baudrate) != 0) {
					PX4_ERR("invalid baudrate: %s", myoptarg);
					return PX4_ERROR;
				}

				break;

			default:
				return print_usage("unknown option");
			}
		}

		if (device == nullptr) {
			return print_usage("no device specified");
		}

		g_dev = new IrCam(device, (unsigned)baudrate);

		if (g_dev == nullptr) {
			PX4_ERR("alloc failed");
			return PX4_ERROR;
		}

		if (g_dev->init() != PX4_OK) {
			delete g_dev;
			g_dev = nullptr;
			return PX4_ERROR;
		}

		return PX4_OK;
	}

	if (!strcmp(argv[1], "selftest")) {
		return selftest();
	}

	if (!strcmp(argv[1], "status")) {
		if (g_dev == nullptr) {
			PX4_INFO("not running");
			return PX4_OK;
		}

		g_dev->print_info();
		return PX4_OK;
	}

	if (!strcmp(argv[1], "stop")) {
		if (g_dev != nullptr) {
			delete g_dev;
			g_dev = nullptr;
		}

		return PX4_OK;
	}

	return print_usage("unknown command");
}
