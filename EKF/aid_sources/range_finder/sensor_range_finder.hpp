/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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
 * @file sensor_range_finder.hpp
 * Range finder class containing all the required checks
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 *
 */
#ifndef EKF_SENSOR_RANGE_FINDER_HPP
#define EKF_SENSOR_RANGE_FINDER_HPP

#include <matrix/math.hpp>
#include <math/filter/MedianFilter.hpp>

namespace estimator
{
namespace sensor
{

struct rangeSample {
	uint64_t    time_us{};  ///< timestamp of the measurement (uSec)
	float       rng{};      ///< range (distance to ground) measurement (m)
	int8_t      quality{};  ///< Signal quality in percent (0...100%), where 0 = invalid signal, 100 = perfect signal, and -1 = unknown signal quality.
};

static constexpr uint64_t RNG_MAX_INTERVAL =
	200e3;  ///< Maximum allowable time interval between range finder measurements (uSec)

/**
 * @brief Buffers range measurements and provides delayed, validated samples.
 *
 * Applies configured sensor limits and timestamp delay handling for range
 * fusion. Distances are in metres and sample times are monotonic microseconds.
 */
class SensorRangeFinder
{
public:
	/** @brief Construct a range sensor validator with default limits and state. */
	SensorRangeFinder() = default;
	/** @brief Destroy the range sensor validator. */
	~SensorRangeFinder() = default;

	/** @brief Evaluate sample freshness, range, quality, tilt, fog, and stuck checks. */
	void runChecks(uint64_t current_time_us, const matrix::Dcmf &R_to_earth);
	/** @brief Return whether the latest range sample passed validity checks. */
	bool isHealthy() const { return _is_sample_valid; }
	/** @brief Return whether a sample is both ready and valid. */
	bool isDataHealthy() const { return _is_sample_ready && _is_sample_valid; }
	/** @brief Return whether a new range sample is ready for processing. */
	bool isDataReady() const { return _is_sample_ready; }
	/** @brief Return whether the sensor is sending samples at a regular rate. */
	bool isRegularlySendingData() const { return _is_regularly_sending_data; }
	/** @brief Return whether the stuck-sensor detector is configured. */
	bool isStuckDetectorEnabled() const { return _stuck_threshold > 0.f; }

	/** @brief Set sample. */
	void setSample(const rangeSample &sample)
	{
		_sample = sample;
		_is_sample_ready = true;
	}

	// This is required because of the ring buffer
	// TODO: move the ring buffer here
	rangeSample *getSampleAddress() { return &_sample; }

	/** @brief Set pitch offset. */
	void setPitchOffset(float new_pitch_offset)
	{
		if (fabsf(_pitch_offset_rad - new_pitch_offset) > FLT_EPSILON) {
			_sin_pitch_offset = sinf(new_pitch_offset);
			_cos_pitch_offset = cosf(new_pitch_offset);
			_pitch_offset_rad = new_pitch_offset;
		}
	}

	/** @brief Set minimum allowed cosine of sensor tilt relative to earth vertical. */
	void setCosMaxTilt(float cos_max_tilt) { _range_cos_max_tilt = cos_max_tilt; }

	/** @brief Set limits. */
	void setLimits(float min_distance, float max_distance)
	{
		_rng_valid_min_val = min_distance;
		_rng_valid_max_val = max_distance;
	}

	/** @brief Set maximum distance treated as a blocked-sensor fog return, in metres. */
	void setMaxFogDistance(float max_fog_dist) { _max_fog_dist = max_fog_dist; }

	/** @brief Set quality hysteresis. */
	void setQualityHysteresis(float valid_quality_threshold_s)
	{
		_quality_hyst_us = uint64_t(valid_quality_threshold_s * 1e6f);
	}

	/** @brief Return cosine of current range-sensor tilt relative to earth vertical. */
	float getCosTilt() const { return _cos_tilt_rng_to_earth; }

	/** @brief Set latest slant range measurement in metres. */
	void setRange(float rng) { _sample.rng = rng; }
	/** @brief Return latest slant range measurement in metres. */
	float getRange() const { return _sample.rng; }

	/** @brief Return range projected onto earth vertical, in metres. */
	float getDistBottom() const { return _sample.rng * _cos_tilt_rng_to_earth; }

	/** @brief Mark whether a new sample is ready for checks. */
	void setDataReadiness(bool is_ready) { _is_sample_ready = is_ready; }
	/** @brief Set the current sample validity decision. */
	void setValidity(bool is_valid) { _is_sample_valid = is_valid; }

	/** @brief Return configured minimum valid range in metres. */
	float getValidMinVal() const { return _rng_valid_min_val; }
	/** @brief Return configured maximum valid range in metres. */
	float getValidMaxVal() const { return _rng_valid_max_val; }

private:
	/** @brief Update sensor-to-earth rotation and projected tilt. */
	void updateSensorToEarthRotation(const matrix::Dcmf &R_to_earth);

	/** @brief Update sample validity from current time and timeout state. */
	void updateValidity(uint64_t current_time_us);
	/** @brief Update filtered inter-sample interval and regularity flag. */
	void updateDtDataLpf(uint64_t current_time_us);
	/** @brief Return whether latest sample is too old at current estimator time. */
	bool isSampleOutOfDate(uint64_t current_time_us) const;
	/** @brief Return whether the filtered sample interval indicates continuity. */
	bool isDataContinuous() const { return _dt_data_lpf < 2e6f; }
	/** @brief Return whether sensor tilt is within the configured limit. */
	bool isTiltOk() const { return _cos_tilt_rng_to_earth > _range_cos_max_tilt; }
	/** @brief Return whether the latest measured distance is inside configured limits. */
	bool isDataInRange() const;
	/** @brief Return whether sensor-reported quality remains acceptable at current time. */
	bool isQualityOk(uint64_t current_time_us) const;
	/** @brief Update stuck-return detection state. */
	void updateStuckCheck();
	/** @brief Update blocked/fog return detection using vertical range and timestamp. */
	void updateFogCheck(const float dist_bottom, const uint64_t time_us);

	rangeSample _sample{}; ///< Stores sample.

	bool _is_sample_ready{};	///< true when new range finder data has fallen behind the fusion time horizon and is available to be fused
	bool _is_sample_valid{};	///< true if range finder sample retrieved from buffer is valid
	bool _is_regularly_sending_data{false}; ///< true if the interval between two samples is less than the maximum expected interval
	uint64_t _time_last_valid_us{};	///< time the last range finder measurement was ready (uSec)

	/*
	 * Stuck check
	 */
	bool _is_stuck{}; ///< State flag indicating is stuck.
	float _stuck_threshold{0.1f};	///< minimum variation in range finder reading required to declare a range finder 'unstuck' when readings recommence after being out of range (m), set to zero to disable
	float _stuck_min_val{};		///< minimum value for new rng measurement when being stuck
	float _stuck_max_val{};		///< maximum value for new rng measurement when being stuck

	/*
	 * Data regularity check
	 */
	static constexpr float _dt_update{0.01f}; 	///< delta time since last ekf update TODO: this should be a parameter
	float _dt_data_lpf{};	///< filtered value of the delta time elapsed since the last range measurement came into the filter (uSec)

	/*
	 * Tilt check
	 */
	float _cos_tilt_rng_to_earth{1.f};		///< 2,2 element of the rotation matrix from sensor frame to earth frame
	float _range_cos_max_tilt{0.7071f};	///< cosine of the maximum tilt angle from the vertical that permits use of range finder and flow data
	float _pitch_offset_rad{3.14f}; 		///< range finder tilt rotation about the Y body axis
	float _sin_pitch_offset{0.0f}; 		///< sine of the range finder tilt rotation about the Y body axis
	float _cos_pitch_offset{-1.0f}; 		///< cosine of the range finder tilt rotation about the Y body axis

	/*
	 * Range check
	 */
	float _rng_valid_min_val{};	///< minimum distance that the rangefinder can measure (m)
	float _rng_valid_max_val{};	///< maximum distance that the rangefinder can measure (m)

	/*
	 * Quality check
	 */
	uint64_t _time_bad_quality_us{};	///< timestamp at which range finder signal quality was 0 (used for hysteresis)
	uint64_t _quality_hyst_us{};		///< minimum duration during which the reported range finder signal quality needs to be non-zero in order to be declared valid (us)

	/*
	 * Fog check
	 */
	bool _is_blocked{false}; ///< State flag indicating is blocked.
	float _max_fog_dist{0.f};	//< maximum distance at which the range finder could detect fog (m)
	math::MedianFilter<float, 5> _median_dist{}; ///< Stores median dist.
	float _prev_median_dist{0.f}; ///< Stores prev median dist.

};

} // namespace sensor
} // namespace estimator
#endif // !EKF_SENSOR_RANGE_FINDER_HPP
