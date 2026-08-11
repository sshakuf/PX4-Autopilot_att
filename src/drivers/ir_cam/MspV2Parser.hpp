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
 * @file MspV2Parser.hpp
 *
 * Byte-at-a-time MSP V2 frame parser for the LSDT IR spot-detection camera.
 * Pure logic, no I/O - testable in isolation. Wire format per the validated
 * project spec (px4_ir_spec.md): '$X' sync, direction, flag, u16 LE function,
 * u16 LE size, payload, CRC-8/DVB-S2 over flag..payload.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

uint8_t crc8_dvb_s2(const uint8_t *data, size_t len, uint8_t crc);

class MspV2Parser
{
public:
	static constexpr uint16_t MAX_PAYLOAD = 256; // SpotsReport max is 229; larger sizes treated as corruption

	struct Frame {
		uint8_t direction;      // '<', '>' or '!'
		uint8_t flag;
		uint16_t function;
		uint16_t size;
		const uint8_t *payload; // points into internal buffer, valid until next parse() call
		bool crc_ok;
	};

	/**
	 * Feed one received byte.
	 * @return true when a complete frame (good or bad CRC) is available in out.
	 */
	bool parse(uint8_t byte, Frame &out);

	uint32_t crcErrors() const { return _crc_errors; }
	uint32_t framesOk() const { return _frames_ok; }

private:
	enum class State : uint8_t {
		IDLE,
		GOT_DOLLAR,
		GOT_X,
		FLAG,
		FUNC_LO,
		FUNC_HI,
		SIZE_LO,
		SIZE_HI,
		PAYLOAD,
		CRC,
	};

	State _state{State::IDLE};
	uint8_t _direction{0};
	uint8_t _flag{0};
	uint16_t _function{0};
	uint16_t _size{0};
	uint16_t _payload_len{0};
	uint8_t _crc{0};
	uint8_t _payload[MAX_PAYLOAD] {};

	uint32_t _crc_errors{0};
	uint32_t _frames_ok{0};
};
