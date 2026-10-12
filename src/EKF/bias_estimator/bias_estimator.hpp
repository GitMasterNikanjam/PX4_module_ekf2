/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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
 * @file bias_estimator.hpp
 * @brief Implementation of a single-state bias estimator
 *
 * state: bias
 *
 * The state is noise driven: Transition matrix A = 1
 * x[k+1] = Ax[k] + v with v ~ N(0, Q)
 * y[k] = x[k] + w with w ~ N(0, R)
 *
 * During the measurment update step, the Normalized Innovation Squared (NIS) is checked
 * and the measurement is rejected if larger than the gate size.
 *
 * @author Mathieu Bresciani 	<mathieu@auterion.com>
 */

#ifndef EKF_BIAS_ESTIMATOR_HPP
#define EKF_BIAS_ESTIMATOR_HPP

#include <matrix/math.hpp>
#include <mathlib.h>
#include <math/filter/AlphaFilter.hpp>

/**
 * @brief Scalar random-walk bias estimator with innovation gating.
 *
 * The bias follows a noise-driven one-state model. predict() grows its
 * uncertainty using the process-noise spectral density, while fuseBias()
 * applies a scalar measurement update when its normalized innovation passes
 * the configured gate. Units are defined by the quantity being estimated.
 */
class BiasEstimator
{
public:
	struct status {
		float bias{0.f}; ///< Stores bias.
		float bias_var{0.f}; ///< Variance or covariance value: bias var.
		float innov{0.f}; ///< Stores innov.
		float innov_var{0.f}; ///< Variance or covariance value: innov var.
		float innov_test_ratio{INFINITY}; ///< Stores innov test ratio.
	};

	/** @brief Construct a scalar bias estimator with default zero state. */
	BiasEstimator() {}
	/** @brief Construct a scalar bias estimator with an initial state and variance. */
	BiasEstimator(float state_init, float state_var_init): _state{state_init}, _state_var{state_var_init} {};

	/** @brief Destroy the estimator instance. */
	virtual ~BiasEstimator() = default;

	/** @brief Reset estimate, covariance, and innovation-monitoring history. */
	void reset()
	{
		_state = 0.f;
		_state_var = 0.f;
		_signed_innov_test_ratio_lpf.reset(0.f);
		_time_since_last_negative_innov = 0.f;
		_time_since_last_positive_innov = 0.f;
	}

	/** @brief Predict bias covariance forward by dt seconds using process noise. */
	virtual void predict(float dt);
	/** @brief Fuse a scalar bias observation when its normalized innovation passes the gate. */
	virtual void fuseBias(float measurement, float measurement_var);

	/** @brief Set bias estimate directly in the estimated quantity's units. */
	void setBias(float bias) { _state = bias; }
	/** @brief Configure process-noise spectral density for bias random walk. */
	void setProcessNoiseSpectralDensity(float nsd)
	{
		_process_psd = nsd * nsd;
	}
	/** @brief Set bias standard deviation; the value is squared internally. */
	void setBiasStdDev(float state_noise) { _state_var = state_noise * state_noise; }
	/** @brief Set normalized innovation gate size in standard deviations. */
	void setInnovGate(float gate_size) { _gate_size = gate_size; }

	/** @brief Set maximum allowed bias standard deviation. */
	void setMaxStateNoise(float max_noise) { _state_var_max = math::max(sq(0.01f), max_noise * max_noise); }

	/** @brief Return current bias estimate. */
	float getBias() const { return _state; }
	/** @brief Return current bias variance. */
	float getBiasVar() const { return _state_var; }
	/** @brief Return latest innovation and estimator status record. */
	const status &getStatus() const { return _status; }

private:
	float _state{0.f}; ///< Stores state.
	float _dt{0.01f}; ///< Time interval or time constant in seconds: dt.

	float _gate_size{3.f}; ///< Used for innovation filtering (innovation test ratio)
	float _state_var{0.1f}; ///< Initial state uncertainty variance (m^2)
	float _process_psd{1.25e-6f}; ///< State process power spectral density (m^2/s^2/Hz)
	float _state_var_max{2.f}; ///< Used to constrain the state variance (m^2)

	// Innovation sequence monitoring; used to detect a bias in the state
	AlphaFilter<float> _signed_innov_test_ratio_lpf; ///< Stores signed innov test ratio lpf.
	float _time_since_last_negative_innov{0.f}; ///< Timestamp or duration value in microseconds: time since last negative innov.
	float _time_since_last_positive_innov{0.f}; ///< Timestamp or duration value in microseconds: time since last positive innov.

	status _status{}; ///< Stores status.

	/** @brief Constrain state var. */
	void constrainStateVar();
	/** @brief Compute kalman gain. */
	float computeKalmanGain(float innov_var) const;

	/*
	 * Compute the ratio between the Normalized Innovation Squared (NIS)
	 * and its maximum gate size. Use isTestRatioPassing to know if the
	 * measurement should be fused or not.
	 */
	/** @brief Compute innov test ratio. */
	float computeInnovTestRatio(float innov, float innov_var) const;
	/** @brief Is test ratio passing. */
	bool isTestRatioPassing(float innov_test_ratio) const;

	/** @brief Update state. */
	void updateState(float K, float innov);
	/** @brief Update state covariance. */
	void updateStateCovariance(float K);

	/** @brief Update offset detection. */
	void updateOffsetDetection(float innov, float innov_test_ratio);
	/** @brief Is offset detected. */
	bool isOffsetDetected() const;

	/** @brief Build the public status record from the latest scalar innovation values. */
	status packStatus(float innov, float innov_var, float innov_test_ratio) const;

	static constexpr float _innov_sequence_monitnoring_time_constant{10.f}; ///< in seconds
	static constexpr float _process_var_boost_gain{1.0e3f}; ///< Compile-time constant defining process var boost gain.
};

#endif // !EKF_BIAS_ESTIMATOR_HPP
