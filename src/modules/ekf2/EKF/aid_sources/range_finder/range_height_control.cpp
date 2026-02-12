/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file range_height_control.cpp
 * Control functions for ekf range finder height fusion
 */

#include "ekf.h"
#include "ekf_derivation/generated/compute_hagl_h.h"
#include "ekf_derivation/generated/compute_hagl_innov_var.h"

void Ekf::controlRangeHaglFusion(const imuSample &imu_sample)
{
	static constexpr const char *HGT_SRC_NAME = "RNG";

	bool rng_data_ready = false; 

	if (_range_buffer) {
		// Get range data from buffer and check validity
		rng_data_ready = _range_buffer->pop_first_older_than(imu_sample.time_us, _range_sensor.getSampleAddress());
		_range_sensor.setDataReadiness(rng_data_ready);

		// ALWAYS log buffer pop status to diagnose issue
		ECL_WARN("RNG: Buffer pop - ready:%d regular:%d imu_time:%llu",
			(int)rng_data_ready,
			(int)_range_sensor.isRegularlySendingData(),
			(unsigned long long)imu_sample.time_us);

		// update range sensor angle parameters in case they have changed
		_range_sensor.setPitchOffset(_params.ekf2_rng_pitch);
		_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
		_range_sensor.setQualityHysteresis(_params.ekf2_rng_qlty_t);
		_range_sensor.setMaxFogDistance(_params.ekf2_rng_fog);

		_range_sensor.runChecks(imu_sample.time_us, _R_to_earth);

		if (_range_sensor.isDataHealthy()) {
			ECL_WARN("RNG: Data healthy, range=%.2fm", (double)_range_sensor.getRange());
			// correct the range data for position offset relative to the IMU
			const Vector3f pos_offset_body = _params.rng_pos_body - _params.imu_pos_body;
			const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;
			_range_sensor.setRange(_range_sensor.getRange() + pos_offset_earth(2) / _range_sensor.getCosTilt());

			if (_control_status.flags.in_air) {
				const bool horizontal_motion = _control_status.flags.fixed_wing
							       || (sq(_state.vel(0)) + sq(_state.vel(1)) > fmaxf(P.trace<2>(State::vel.idx), 0.1f));

				const float dist_dependant_var = sq(_params.ekf2_rng_sfe * _range_sensor.getDistBottom());
				const float var = sq(_params.ekf2_rng_noise) + dist_dependant_var;

				_rng_consistency_check.setGate(_params.ekf2_rng_k_gate);
				_rng_consistency_check.update(_range_sensor.getDistBottom(), math::max(var, 0.001f), _state.vel(2),
							      P(State::vel.idx + 2, State::vel.idx + 2), horizontal_motion, imu_sample.time_us);
			}

		} else {
			// If we are supposed to be using range finder data but have bad range measurements
			// and are on the ground, then synthesise a measurement at the expected on ground value
			if (!_control_status.flags.in_air
			    && _control_status.flags.vehicle_at_rest
			    && _range_sensor.isRegularlySendingData()
			    && _range_sensor.isDataReady()) {

				_range_sensor.setRange(_params.ekf2_min_rng);
				_range_sensor.setValidity(true); // bypass the checks
				ECL_INFO("RNG: On ground, synthesizing range=%.2fm", (double)_params.ekf2_min_rng);
			} else {
				ECL_WARN("RNG: Data unhealthy - ready:%d regular:%d valid:%d",
					_range_sensor.isDataReady(),
					_range_sensor.isRegularlySendingData(),
					_range_sensor.isHealthy());
			}
		}

		_control_status.flags.rng_kin_consistent = _rng_consistency_check.isKinematicallyConsistent();

	} else {
		return;
	}

	auto &aid_src = _aid_src_rng_hgt;

	ECL_WARN("RNG: Pre-fusion check - ready:%d has_addr:%d",
		(int)rng_data_ready,
		(int)(_range_sensor.getSampleAddress() != nullptr));

	if (rng_data_ready && _range_sensor.getSampleAddress()) {
		ECL_WARN("RNG: Inside fusion logic - starting analysis");

		const float measurement = math::max(_range_sensor.getDistBottom(), _params.ekf2_min_rng);
		const float measurement_variance = getRngVar();

		float innovation_variance;
		sym::ComputeHaglInnovVar(P, measurement_variance, &innovation_variance);

		const float innov_gate = math::max(_params.ekf2_rng_gate, 1.f);
		updateAidSourceStatus(aid_src,
				      _range_sensor.getSampleAddress()->time_us, // sample timestamp
				      measurement,                               // observation
				      measurement_variance,                      // observation variance
				      getHagl() - measurement,                   // innovation
				      innovation_variance,                       // innovation variance
				      innov_gate);                               // innovation gate

		const bool measurement_valid = PX4_ISFINITE(aid_src.observation) && PX4_ISFINITE(aid_src.observation_variance);

		// z special case if there is bad vertical acceleration data, then don't reject measurement,
		// but limit innovation to prevent spikes that could destabilise the filter
		if (_fault_status.flags.bad_acc_vertical && aid_src.innovation_rejected
		    && measurement_valid && _range_sensor.isDataHealthy()
		   ) {
			const float innov_limit = innov_gate * sqrtf(aid_src.innovation_variance);
			aid_src.innovation = math::constrain(aid_src.innovation, -innov_limit, innov_limit);
			aid_src.innovation_rejected = false;
		}

		const bool continuing_conditions_passing = ((_params.ekf2_rng_ctrl == static_cast<int32_t>(RngCtrl::ENABLED))
				|| (_params.ekf2_rng_ctrl == static_cast<int32_t>(RngCtrl::CONDITIONAL)))
				&& _control_status.flags.tilt_align
				&& measurement_valid;

		const bool starting_conditions_passing = continuing_conditions_passing
				&& isNewestSampleRecent(_time_last_range_buffer_push, 2 * estimator::sensor::RNG_MAX_INTERVAL)
				&& _range_sensor.isRegularlySendingData()
				&& _range_sensor.isDataHealthy();

		const bool rng_flags_ok = (_control_status.flags.rng_terrain || _control_status.flags.rng_hgt);
		const bool is_conditional_mode = (_params.ekf2_rng_ctrl == static_cast<int32_t>(RngCtrl::CONDITIONAL));
		const bool is_suitable = isConditionalRangeAidSuitable();

		const bool do_conditional_range_aid = rng_flags_ok && is_conditional_mode && is_suitable;

		const bool do_range_aid = (_control_status.flags.rng_terrain || _control_status.flags.rng_hgt)
					  && (_params.ekf2_rng_ctrl == static_cast<int32_t>(RngCtrl::ENABLED));

		// Diagnostic: Log why conditional range aid is failing
		if (rng_flags_ok && !do_conditional_range_aid) {
			ECL_WARN("RNG: Conditional aid FALSE - mode:%d suitable:%d ctrl_val:%d",
				(int)is_conditional_mode, (int)is_suitable, (int)_params.ekf2_rng_ctrl);
		}

		// Log why starting conditions fail
		if (!_control_status.flags.rng_hgt && !starting_conditions_passing) {
			ECL_WARN("RNG: Starting conditions FAIL - ctrl:%d tilt:%d meas_valid:%d recent:%d regular:%d healthy:%d",
				(int)_params.ekf2_rng_ctrl,
				(int)_control_status.flags.tilt_align,
				(int)measurement_valid,
				(int)isNewestSampleRecent(_time_last_range_buffer_push, 2 * estimator::sensor::RNG_MAX_INTERVAL),
				(int)_range_sensor.isRegularlySendingData(),
				(int)_range_sensor.isDataHealthy());
		}

		if (_control_status.flags.rng_hgt) {
			if (!(do_conditional_range_aid || do_range_aid)) {
				ECL_INFO("stopping %s fusion", HGT_SRC_NAME);
				stopRngHgtFusion();
			}

		} else if (starting_conditions_passing) {
			ECL_WARN("RNG: In startup section - hgt_ref:%d is_range:%d",
				(int)_params.ekf2_hgt_ref,
				(int)(_params.ekf2_hgt_ref == static_cast<int32_t>(HeightSensor::RANGE)));

			if (_params.ekf2_hgt_ref == static_cast<int32_t>(HeightSensor::RANGE)) {
				if (do_conditional_range_aid) {
					// Range finder is used while hovering to stabilize the height estimate. Don't reset but use it as height reference.
					ECL_INFO("starting conditional %s height fusion", HGT_SRC_NAME);
					_height_sensor_ref = HeightSensor::RANGE;

					_control_status.flags.rng_hgt = true;
					stopRngTerrFusion();

					if (!_control_status.flags.opt_flow_terrain && aid_src.innovation_rejected) {
						resetTerrainToRng(aid_src);
						resetAidSourceStatusZeroInnovation(aid_src);
					}

				} else if (do_range_aid) {
					// Range finder is the primary height source, the ground is now the datum used
					// to compute the local vertical position
					ECL_INFO("starting %s height fusion, resetting height", HGT_SRC_NAME);
					_height_sensor_ref = HeightSensor::RANGE;

					_information_events.flags.reset_hgt_to_rng = true;
					resetAltitudeTo(aid_src.observation, aid_src.observation_variance);
					_state.terrain = 0.f;
					resetAidSourceStatusZeroInnovation(aid_src);
					_control_status.flags.rng_hgt = true;
					stopRngTerrFusion();

					aid_src.time_last_fuse = imu_sample.time_us;
				}

			} else {
				ECL_WARN("RNG: Height ref NOT range - do_cond:%d do_aid:%d",
					(int)do_conditional_range_aid,
					(int)do_range_aid);

				if (do_conditional_range_aid || do_range_aid) {
					ECL_WARN("RNG: Setting rng_hgt=true in startup");
					_control_status.flags.rng_hgt = true;

					// IMPORTANT: Also enable terrain fusion for optical flow
					// Without this, terrain never becomes valid and optical flow can't start
					if (!_control_status.flags.rng_terrain) {
						ECL_WARN("RNG: ALSO enabling terrain fusion for optical flow support");
						_control_status.flags.rng_terrain = true;
					}

					if (!_control_status.flags.opt_flow_terrain && aid_src.innovation_rejected) {
						ECL_WARN("RNG: Also resetting terrain");
						resetTerrainToRng(aid_src);
						resetAidSourceStatusZeroInnovation(aid_src);
					}
				} else {
					ECL_WARN("RNG: NOT setting rng_hgt - both do_conditional and do_aid are false");
				}
			}
		}

		ECL_WARN("RNG: Fusion status check - rng_hgt:%d rng_terrain:%d start_cond:%d cont_cond:%d",
			(int)_control_status.flags.rng_hgt,
			(int)_control_status.flags.rng_terrain,
			(int)starting_conditions_passing,
			(int)continuing_conditions_passing);

		if (_control_status.flags.rng_hgt || _control_status.flags.rng_terrain) {
			ECL_WARN("RNG: One or both fusion types active - managing continuing fusion");

			// Fix: If height fusion is active but terrain fusion isn't, enable terrain fusion
			// This supports optical flow which requires terrain validity
			if (_control_status.flags.rng_hgt && !_control_status.flags.rng_terrain) {
				ECL_WARN("RNG: Height fusion active but terrain fusion disabled - ENABLING terrain fusion");
				_control_status.flags.rng_terrain = true;
			}

			if (continuing_conditions_passing) {

				if (do_conditional_range_aid) {
					_height_sensor_ref = HeightSensor::RANGE;

				} else if (_height_sensor_ref == HeightSensor::RANGE) {
					_height_sensor_ref = HeightSensor::UNKNOWN;
				}

				if (_range_sensor.isDataHealthy()
				    && _control_status.flags.rng_kin_consistent
				   ) {
					fuseHaglRng(aid_src, _control_status.flags.rng_hgt, _control_status.flags.rng_terrain);
				}

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired()
				    && _control_status.flags.rng_hgt
				    && (_height_sensor_ref == HeightSensor::RANGE)
				    && starting_conditions_passing
				   ) {
					// All height sources are failing
					ECL_WARN("%s height fusion reset required, all height sources failing", HGT_SRC_NAME);

					_information_events.flags.reset_hgt_to_rng = true;
					resetAltitudeTo(aid_src.observation - _state.terrain);
					resetAidSourceStatusZeroInnovation(aid_src);

					// reset vertical velocity if no valid sources available
					if (!isVerticalVelocityAidingActive()) {
						resetVerticalVelocityToZero();
					}

					aid_src.time_last_fuse = imu_sample.time_us;

				} else if (is_fusion_failing) {
					// Some other height source is still working
					if (_control_status.flags.opt_flow_terrain && isTerrainEstimateValid()) {
						ECL_WARN("stopping %s fusion, fusion failing", HGT_SRC_NAME);
						stopRngHgtFusion();
						stopRngTerrFusion();

					} else if (starting_conditions_passing) {
						resetTerrainToRng(aid_src);
						resetAidSourceStatusZeroInnovation(aid_src);
					}
				}

			} else {
				ECL_WARN("stopping %s fusion, continuing conditions failing", HGT_SRC_NAME);
				stopRngHgtFusion();
				stopRngTerrFusion();
			}

		} else {
			ECL_WARN("RNG: Not active yet - rng_hgt:%d rng_terrain:%d start_cond:%d",
				(int)_control_status.flags.rng_hgt,
				(int)_control_status.flags.rng_terrain,
				(int)starting_conditions_passing);

			if (starting_conditions_passing) {
				if (_control_status.flags.opt_flow_terrain) {
					ECL_WARN("RNG: Starting terrain fusion with optical flow active");
					if (!aid_src.innovation_rejected) {
						_control_status.flags.rng_terrain = true;
						fuseHaglRng(aid_src, _control_status.flags.rng_hgt, _control_status.flags.rng_terrain);
						ECL_WARN("RNG: Terrain fusion started with optical flow");
					} else {
						ECL_WARN("RNG: Innovation rejected, not starting");
					}

				} else {
					ECL_WARN("RNG: Starting terrain fusion without optical flow");
					if (aid_src.innovation_rejected) {
						ECL_WARN("RNG: Innovation rejected, resetting terrain");
						resetTerrainToRng(aid_src);
						resetAidSourceStatusZeroInnovation(aid_src);
					}

					_control_status.flags.rng_terrain = true;
					ECL_WARN("RNG: Terrain fusion enabled, rng_terrain=%d", (int)_control_status.flags.rng_terrain);
				}
			} else {
				ECL_WARN("RNG: Terrain fusion not started - starting conditions not passing");
			}
		}

	} else if ((_control_status.flags.rng_hgt || _control_status.flags.rng_terrain)
		   && !isNewestSampleRecent(_time_last_range_buffer_push, 2 * estimator::sensor::RNG_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		ECL_WARN("stopping %s fusion, no data", HGT_SRC_NAME);
		stopRngHgtFusion();
		stopRngTerrFusion();
	}
}

float Ekf::getRngVar() const
{
	return fmaxf(
		       P(State::pos.idx + 2, State::pos.idx + 2)
		       + sq(_params.ekf2_rng_noise)
		       + sq(_params.ekf2_rng_sfe * _range_sensor.getRange()),
		       0.f);
}

void Ekf::resetTerrainToRng(estimator_aid_source1d_s &aid_src)
{
	// Since the distance is not a direct observation of the terrain state but is based
	// on the height state, a reset should consider the height uncertainty. This can be
	// done by manipulating the Kalman gain to inject all the innovation in the terrain state
	// and create the correct correlation with the terrain state with a covariance update.
	P.uncorrelateCovarianceSetVariance<State::terrain.dof>(State::terrain.idx, 0.f);

	const float old_terrain = _state.terrain;

	VectorState H;
	sym::ComputeHaglH(&H);

	VectorState K;
	K(State::terrain.idx) = 1.f; // innovation is forced into the terrain state to create a "reset"

	measurementUpdate(K, H, aid_src.observation_variance, aid_src.innovation);

	// record the state change
	const float delta_terrain = _state.terrain - old_terrain;

	if (_state_reset_status.reset_count.hagl == _state_reset_count_prev.hagl) {
		_state_reset_status.hagl_change = delta_terrain;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.hagl_change += delta_terrain;
	}

	_state_reset_status.reset_count.hagl++;

	aid_src.time_last_fuse = _time_delayed_us;
}

bool Ekf::isConditionalRangeAidSuitable()
{
	// check if we can use range finder measurements to estimate height, use hysteresis to avoid rapid switching
	// Note that the 0.7 coefficients and the innovation check are arbitrary values but work well in practice
	float range_hagl_max = _params.ekf2_rng_a_hmax;
	float max_vel_xy = _params.ekf2_rng_a_vmax;

	const float hagl_test_ratio = _aid_src_rng_hgt.test_ratio;

	bool is_hagl_stable = (hagl_test_ratio < 1.f);

	if (!_control_status.flags.rng_hgt) {
		range_hagl_max = 0.7f * _params.ekf2_rng_a_hmax;
		max_vel_xy = 0.7f * _params.ekf2_rng_a_vmax;
		is_hagl_stable = (hagl_test_ratio < 0.01f);
	}

	const bool is_in_range = (getHagl() < range_hagl_max);

	bool is_below_max_speed = true;

	if (isHorizontalAidingActive()) {
		is_below_max_speed = !_state.vel.xy().longerThan(max_vel_xy);
	}

	const bool result = is_in_range && is_hagl_stable && is_below_max_speed;

	// Diagnostic: Log why conditional range aid suitability check fails
	if (!result) {
		ECL_WARN("RNG: Conditional NOT suitable - in_range:%d (%.2f<%.2f) stable:%d (%.2f<%.2f) speed_ok:%d vel:%.2f",
			(int)is_in_range, (double)getHagl(), (double)range_hagl_max,
			(int)is_hagl_stable, (double)hagl_test_ratio, (double)(_control_status.flags.rng_hgt ? 1.0f : 0.01f),
			(int)is_below_max_speed, (double)_state.vel.xy().norm());
	}

	return result;
}

void Ekf::stopRngHgtFusion()
{
	if (_control_status.flags.rng_hgt) {

		if (_height_sensor_ref == HeightSensor::RANGE) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_control_status.flags.rng_hgt = false;
	}
}

void Ekf::stopRngTerrFusion()
{
	_control_status.flags.rng_terrain = false;
}
