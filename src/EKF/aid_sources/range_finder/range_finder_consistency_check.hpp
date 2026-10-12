/****************************************************************************
 *
 *   Copyright (c) 2022 PX4. All rights reserved.
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
 * @file range_finder_consistency_check.hpp
 * @brief Compute statistical tests of the range finder data
 * using the estimated velocity as a reference in order to detect sensor faults
 */

#ifndef EKF_RANGE_FINDER_CONSISTENCY_CHECK_HPP
#define EKF_RANGE_FINDER_CONSISTENCY_CHECK_HPP

#include <math/filter/AlphaFilter.hpp>

/**
 * @brief Checks range-derived vertical motion against inertial predictions.
 *
 * Filtered innovations and test ratios expose whether range data is
 * kinematically consistent enough for fusion. Distances use metres and update
 * timing follows the estimator's microsecond clock.
 */
class RangeFinderConsistencyCheck final
{
public:
	/** @brief Construct an uninitialized range consistency checker. */
	RangeFinderConsistencyCheck() = default;
	/** @brief Destroy the consistency checker. */
	~RangeFinderConsistencyCheck() = default;

	/** @brief Update range/inertial vertical-motion consistency using metre and second units. */
	void update(float dist_bottom, float dist_bottom_var, float vz, float vz_var, bool horizontal_motion, uint64_t time_us);

	/** @brief Set normalized consistency gate size. */
	void setGate(float gate) { _gate = gate; }

	/** @brief Return current normalized consistency-test ratio. */
	float getTestRatio() const { return _test_ratio; }
	/** @brief Return filtered signed consistency-test ratio. */
	float getSignedTestRatioLpf() const { return _signed_test_ratio_lpf.getState(); }
	/** @brief Return current range/inertial innovation in metres. */
	float getInnov() const { return _innov; }
	/** @brief Return current innovation variance in square metres. */
	float getInnovVar() const { return _innov_var; }
	/** @brief Return whether the latest consistency decision passes. */
	bool isKinematicallyConsistent() const { return _is_kinematically_consistent; }

private:
	/** @brief Update consistency timing and decision using vertical speed in m/s. */
	void updateConsistency(float vz, uint64_t time_us);

	uint64_t _time_last_update_us{}; ///< Timestamp or duration value in microseconds: time last update us.
	float _dist_bottom_prev{}; ///< Stores dist bottom prev.

	float _test_ratio{}; ///< Stores test ratio.
	AlphaFilter<float> _signed_test_ratio_lpf{}; // average signed test ratio used to detect a bias in the data
	float _gate{.2f}; ///< Stores gate.
	float _innov{}; ///< Stores innov.
	float _innov_var{}; ///< Variance or covariance value: innov var.

	bool _is_kinematically_consistent{true}; ///< State flag indicating is kinematically consistent.
	uint64_t _time_last_inconsistent_us{}; ///< Timestamp or duration value in microseconds: time last inconsistent us.
	uint64_t _time_last_horizontal_motion{}; ///< Timestamp or duration value in microseconds: time last horizontal motion.

	static constexpr float _signed_test_ratio_tau = 2.f; ///< Compile-time constant defining signed test ratio tau.

	static constexpr float _min_vz_for_valid_consistency = .5f; ///< Compile-time constant defining min vz for valid consistency.
	static constexpr uint64_t _consistency_hyst_time_us = 1e6; ///< Compile-time constant defining consistency hyst time us.
};

#endif // !EKF_RANGE_FINDER_CONSISTENCY_CHECK_HPP
