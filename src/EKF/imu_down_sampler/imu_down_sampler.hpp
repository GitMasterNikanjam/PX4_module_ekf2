/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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
 * Downsamples IMU data to a lower rate such that EKF predicition can happen less frequent
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */
#ifndef EKF_IMU_DOWN_SAMPLER_HPP
#define EKF_IMU_DOWN_SAMPLER_HPP

#include <mathlib.h>
#include <src/math.hpp>

#include "../common.h"

using namespace estimator;

/**
 * @brief Combines high-rate IMU increments into the EKF prediction interval.
 *
 * Angular and velocity increments are accumulated with their elapsed times;
 * update() indicates when a complete down-sampled output is ready. The target
 * interval is provided through the referenced configuration value.
 */
class ImuDownSampler
{
public:
	/** @brief Construct sampler using a referenced target output period in microseconds. */
	explicit ImuDownSampler(int32_t &target_dt_us);
	/** @brief Destroy the sampler and release its owned resources. */
	~ImuDownSampler() = default;

	/** @brief Accumulate one IMU sample; return true when a down-sampled sample is ready. */
	bool update(const imuSample &imu_sample_new);

	imuSample getDownSampledImuAndTriggerReset()
	{
		imuSample imu{_imu_down_sampled}; ///< Stores imu.
		reset();
		return imu;
	}

private:
	/** @brief Clear accumulated IMU increments and timing state. */
	void reset();

	imuSample _imu_down_sampled{}; ///< Stores imu down sampled.
	Quatf _delta_angle_accumulated{}; ///< Stores delta angle accumulated.

	int _accumulated_samples{0}; ///< Stores accumulated samples.
	int _required_samples{1}; ///< Stores required samples.

	int32_t &_target_dt_us; ///< Timestamp or duration value in microseconds: target dt us.

	float _target_dt_s{0.010f}; ///< Time interval or time constant in seconds: target dt s.
	float _min_dt_s{0.005f}; ///< Time interval or time constant in seconds: min dt s.

	float _delta_ang_dt_avg{0.005f}; ///< Time interval or time constant in seconds: delta ang dt avg.
};
#endif // !EKF_IMU_DOWN_SAMPLER_HPP
