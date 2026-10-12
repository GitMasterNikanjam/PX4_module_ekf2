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

#ifndef EKF_AUX_GLOBAL_POSITION_CONTROL_HPP
#define EKF_AUX_GLOBAL_POSITION_CONTROL_HPP

/**
 * @file aux_global_position_control.hpp
 * @brief PX4 uORB source adapter for auxiliary global-position fusion.
 *
 * Defines per-slot buffering and publication support used by
 * AuxGlobalPosition. This code requires PX4 uORB and parameter facilities and
 * is enabled only in the corresponding PX4 module build.
 */

#include "../../common.h"

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)

#include <lib/parameters/param.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/estimator_aid_source2d.h>
#include <uORB/topics/aux_global_position.h>

class Ekf;

/**
 * @brief Buffers and fuses one auxiliary global-position sensor instance.
 *
 * The adapter converts PX4 topic samples into delayed EKF observations and
 * publishes aid-source diagnostics. It is a PX4 integration component rather
 * than a standalone host API.
 */
class AgpSource
{
public:
	/** @brief Construct one PX4 auxiliary global-position source for the given slot. */
	AgpSource(int slot);
	~AgpSource() = default;

	/** @brief Buffer a PX4 topic observation using the delayed IMU sample for alignment. */
	void bufferData(const aux_global_position_s &msg, const estimator::imuSample &imu_delayed);
	/** @brief Process buffered data and attempt fusion into the EKF. */
	bool update(Ekf &ekf, const estimator::imuSample &imu_delayed);
	/** @brief Advertise this source's PX4 aid-source diagnostic topic. */
	void advertise() { _aid_src_pub.advertise(); }
	/** @brief Define source-specific PX4 parameters. */
	void initParams();
	/** @brief Refresh cached source parameters from PX4 parameter storage. */
	void updateParams();

	/** @brief Return filtered normalized innovation test ratio. */
	float getTestRatioFiltered() const { return _test_ratio_filtered; }
	/** @brief Return whether this source is actively fusing. */
	bool isFusing() const { return _state == State::kActive; }

private:
	struct AuxGlobalPositionSample {
		uint64_t time_us{}; ///< Timestamp or duration value in microseconds: time us.
		uint8_t id{}; ///< Stores id.
		double latitude{}; ///< Stores latitude.
		double longitude{}; ///< Stores longitude.
		float altitude_amsl{}; ///< Stores altitude amsl.
		float eph{}; ///< Stores eph.
		float epv{}; ///< Stores epv.
		uint8_t lat_lon_reset_counter{}; ///< Stores lat lon reset counter.
	};

	enum class Ctrl : uint8_t {
		kHPos  = (1 << 0),
		kVPos  = (1 << 1)
	};

	enum class Mode : uint8_t {
		kAuto           = 0,   	///< Reset on fusion timeout if no other source of position is available
		kDeadReckoning = 1   	///< Reset on fusion timeout if no source of velocity is available
	};

	enum class State {
		kStopped,
		kStarting,
		kActive,
	};

	struct reset_counters_s {
		uint8_t lat_lon{}; ///< Stores lat lon.
	};

	uORB::PublicationMulti<estimator_aid_source2d_s> _aid_src_pub{ORB_ID(estimator_aid_src_aux_global_position)};

	TimestampedRingBuffer<AuxGlobalPositionSample> _buffer{20}; ///< Buffer used to retain buffer.
	State _state{State::kStopped}; ///< Stores state.
	estimator_aid_source2d_s _aid_src{}; ///< Innovation and fusion status for .
	float _test_ratio_filtered{0.f}; ///< Stores test ratio filtered.
	uint64_t _time_last_buffer_push{0}; ///< Timestamp or duration value in microseconds: time last buffer push.
	reset_counters_s _reset_counters{}; ///< Stores reset counters.
	int _slot; ///< Stores slot.

	struct ParamHandles {
		param_t ctrl{PARAM_INVALID}; ///< Stores ctrl.
		param_t mode{PARAM_INVALID}; ///< Stores mode.
		param_t delay{PARAM_INVALID}; ///< Stores delay.
		param_t noise{PARAM_INVALID}; ///< Noise tuning value: noise.
		param_t gate{PARAM_INVALID}; ///< Stores gate.
	} _param_handles;

	struct Params {
		int32_t ctrl{0}; ///< Stores ctrl.
		int32_t mode{0}; ///< Stores mode.
		float delay{0.f}; ///< Stores delay.
		float noise{10.f}; ///< Noise tuning value: noise.
		float gate{3.f}; ///< Stores gate.
	} _params;

	/** @brief Check whether current EKF conditions permit resetting to this source. */
	bool isResetAllowed(const Ekf &ekf) const;
	/** @brief Check whether sensor data exceeded the supplied timeout period. */
	bool isTimedOut(uint64_t last_sensor_timestamp, uint64_t time_delayed_us, uint64_t timeout_period) const;
};

#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION && MODULE_NAME

#endif // !EKF_AUX_GLOBAL_POSITION_CONTROL_HPP
