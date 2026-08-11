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

#include "MspV2Parser.hpp"

uint8_t crc8_dvb_s2(const uint8_t *data, size_t len, uint8_t crc)
{
	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];

		for (int bit = 0; bit < 8; bit++) {
			if (crc & 0x80) {
				crc = (uint8_t)((crc << 1) ^ 0xD5);

			} else {
				crc = (uint8_t)(crc << 1);
			}
		}
	}

	return crc;
}

bool MspV2Parser::parse(uint8_t byte, Frame &out)
{
	switch (_state) {
	case State::IDLE:
		if (byte == '$') {
			_state = State::GOT_DOLLAR;
		}

		break;

	case State::GOT_DOLLAR:
		_state = (byte == 'X') ? State::GOT_X : State::IDLE;
		break;

	case State::GOT_X:
		if (byte == '<' || byte == '>' || byte == '!') {
			_direction = byte;
			_crc = 0;
			_payload_len = 0;
			_state = State::FLAG;

		} else {
			_state = State::IDLE;
		}

		break;

	case State::FLAG:
		_flag = byte;
		_crc = crc8_dvb_s2(&byte, 1, _crc);
		_state = State::FUNC_LO;
		break;

	case State::FUNC_LO:
		_function = byte;
		_crc = crc8_dvb_s2(&byte, 1, _crc);
		_state = State::FUNC_HI;
		break;

	case State::FUNC_HI:
		_function |= (uint16_t)byte << 8;
		_crc = crc8_dvb_s2(&byte, 1, _crc);
		_state = State::SIZE_LO;
		break;

	case State::SIZE_LO:
		_size = byte;
		_crc = crc8_dvb_s2(&byte, 1, _crc);
		_state = State::SIZE_HI;
		break;

	case State::SIZE_HI:
		_size |= (uint16_t)byte << 8;
		_crc = crc8_dvb_s2(&byte, 1, _crc);

		if (_size > MAX_PAYLOAD) {
			// larger than anything we handle: treat as corruption and resync
			_state = State::IDLE;

		} else {
			_state = (_size > 0) ? State::PAYLOAD : State::CRC;
		}

		break;

	case State::PAYLOAD:
		_payload[_payload_len++] = byte;
		_crc = crc8_dvb_s2(&byte, 1, _crc);

		if (_payload_len == _size) {
			_state = State::CRC;
		}

		break;

	case State::CRC: {
			out.direction = _direction;
			out.flag = _flag;
			out.function = _function;
			out.size = _size;
			out.payload = _payload;
			out.crc_ok = (byte == _crc);

			if (out.crc_ok) {
				_frames_ok++;

			} else {
				_crc_errors++;
			}

			_state = State::IDLE;
			return true;
		}
	}

	return false;
}
