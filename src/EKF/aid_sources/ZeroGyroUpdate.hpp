/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#ifndef EKF_ZERO_GYRO_UPDATE_HPP
#define EKF_ZERO_GYRO_UPDATE_HPP

#include "EstimatorAidSource.hpp"

/**
 * @brief Applies a zero-angular-rate constraint when the vehicle is stationary.
 *
 * The update uses delayed IMU increments and estimator rest/validity logic to
 * constrain gyro bias. It returns whether a measurement update was applied.
 */
class ZeroGyroUpdate : public EstimatorAidSource
{
public:
	/** @brief Construct zero-angular-rate aiding state. */
	ZeroGyroUpdate();
	virtual ~ZeroGyroUpdate() = default;

	/** @brief Clear accumulated zero-rate update history. */
	void reset() override;
	/** @brief Apply zero-angular-rate constraints when stationary conditions pass. */
	bool update(Ekf &ekf, const estimator::imuSample &imu_delayed) override;

private:

	matrix::Vector3f _zgup_delta_ang{}; ///< Stores zgup delta ang.
	float _zgup_delta_ang_dt{0.f}; ///< Time interval or time constant in seconds: zgup delta ang dt.
};

#endif // !EKF_ZERO_GYRO_UPDATE_HPP
