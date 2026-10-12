/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#ifndef EKF_GNSS_CHECKS_H
#define EKF_GNSS_CHECKS_H

#include <geo.h>

#include "../../common.h"

namespace estimator
{
/**
 * @brief Evaluates GNSS quality, drift, and integrity before aiding is enabled.
 *
 * run() updates per-check failure flags and filtered motion estimates using
 * the supplied GNSS sample and current estimator time. passed() reports the
 * current health decision; initialChecksPassed() records whether initial
 * acceptance checks have completed successfully.
 */
class GnssChecks final
{
public:
	/** @brief Construct health checks bound to GNSS limits, timing, and EKF status. */
	GnssChecks(int32_t &check_mask, int32_t &ekf2_req_nsats, float &ekf2_req_pdop, float &ekf2_req_eph, float &ekf2_req_epv,
		   float &ekf2_req_sacc, float &ekf2_req_hdrift, float &ekf2_req_vdrift, int32_t &ekf2_req_fix, float &ekf2_vel_lim,
		   uint32_t &min_health_time_us, filter_control_status_u &control_status):
		_params{check_mask, ekf2_req_nsats, ekf2_req_pdop, ekf2_req_eph, ekf2_req_epv, ekf2_req_sacc, ekf2_req_hdrift, ekf2_req_vdrift, ekf2_req_fix, ekf2_vel_lim, min_health_time_us},
		_control_status(control_status)
	{};

	union gps_check_fail_status_u {
		struct {
			uint16_t fix    : 1; ///< 0 - true if the fix type is insufficient (no 3D solution)
			uint16_t nsats  : 1; ///< 1 - true if number of satellites used is insufficient
			uint16_t pdop   : 1; ///< 2 - true if position dilution of precision is insufficient
			uint16_t hacc   : 1; ///< 3 - true if reported horizontal accuracy is insufficient
			uint16_t vacc   : 1; ///< 4 - true if reported vertical accuracy is insufficient
			uint16_t sacc   : 1; ///< 5 - true if reported speed accuracy is insufficient
			uint16_t hdrift : 1; ///< 6 - true if horizontal drift is excessive (can only be used when stationary on ground)
			uint16_t vdrift : 1; ///< 7 - true if vertical drift is excessive (can only be used when stationary on ground)
			uint16_t hspeed : 1; ///< 8 - true if horizontal speed is excessive (can only be used when stationary on ground)
			uint16_t vspeed : 1; ///< 9 - true if vertical speed error is excessive
			uint16_t spoofed: 1; ///< 10 - true if the GNSS data is spoofed
			uint16_t jammed : 1; ///< 11 - true if the GNSS data is jammed
		} flags;
		uint16_t value; ///< Stores value.
	};

	/** @brief Reset hard. */
	void resetHard()
	{
		_initial_checks_passed = false;
		reset();
	}

	/** @brief Clear pass/fail history while retaining configured GNSS thresholds. */
	void reset()
	{
		_passed = false;
		_time_last_pass_us = 0;
		_time_last_fail_us = 0;
		resetDriftFilters();
	}

	/*
	 * Return true if the GNSS solution quality is adequate.
	*/
	/** @brief Evaluate configured GNSS health checks and update health state. */
	bool run(const gnssSample &gnss, uint64_t time_us);
	/** @brief Return whether all currently enabled GNSS health checks pass. */
	bool passed() const { return _passed; }
	/** @brief Return whether the initial GNSS acceptance checks have passed. */
	bool initialChecksPassed() const { return _initial_checks_passed; }
	/** @brief Return timestamp of latest successful health check in microseconds. */
	uint64_t getLastPassUs() const { return _time_last_pass_us; }
	/** @brief Return timestamp of latest failed health check in microseconds. */
	uint64_t getLastFailUs() const { return _time_last_fail_us; }

	const gps_check_fail_status_u &getFailStatus() const { return _check_fail_status; }

	/** @brief Return filtered horizontal position drift rate in m/s. */
	float horizontal_position_drift_rate_m_s() const { return _horizontal_position_drift_rate_m_s; }
	/** @brief Return filtered vertical position drift rate in m/s. */
	float vertical_position_drift_rate_m_s() const { return _vertical_position_drift_rate_m_s; }
	/** @brief Return filtered horizontal velocity magnitude in m/s. */
	float filtered_horizontal_velocity_m_s() const { return _filtered_horizontal_velocity_m_s; }

private:
	enum class GnssChecksMask : int32_t {
		kNsats   = (1 << 0),
		kPdop    = (1 << 1),
		kHacc    = (1 << 2),
		kVacc    = (1 << 3),
		kSacc    = (1 << 4),
		kHdrift  = (1 << 5),
		kVdrift  = (1 << 6),
		kHspd    = (1 << 7),
		kVspd    = (1 << 8),
		kSpoofed = (1 << 9),
		kFix     = (1 << 10),
		kJammed  = (1 << 11)
	};

	/** @brief Is check enabled. */
	bool isCheckEnabled(GnssChecksMask check) { return (_params.check_mask & static_cast<int32_t>(check)); }

	/** @brief Run simplified checks. */
	bool runSimplifiedChecks(const gnssSample &gnss);
	/** @brief Run initial fix checks. */
	bool runInitialFixChecks(const gnssSample &gnss);
	/** @brief Run on ground gnss checks. */
	void runOnGroundGnssChecks(const gnssSample &gnss);

	/** @brief Reset drift filters. */
	void resetDriftFilters();

	/** @brief Is timed out. */
	bool isTimedOut(uint64_t timestamp_to_check_us, uint64_t now_us, uint64_t timeout_period) const
	{
		return (timestamp_to_check_us == 0) || (timestamp_to_check_us + timeout_period < now_us);
	}

	gps_check_fail_status_u _check_fail_status{}; ///< Stores check fail status.

	float _horizontal_position_drift_rate_m_s{NAN}; ///< Time interval or time constant in seconds: horizontal position drift rate m s.
	float _vertical_position_drift_rate_m_s{NAN}; ///< Time interval or time constant in seconds: vertical position drift rate m s.
	float _filtered_horizontal_velocity_m_s{NAN}; ///< Time interval or time constant in seconds: filtered horizontal velocity m s.

	MapProjection lat_lon_prev{}; ///< Stores lat lon prev.
	float _alt_prev{0.0f}; ///< Stores alt prev.

	Vector3f _lat_lon_alt_deriv_filt{}; ///< Stores lat lon alt deriv filt.
	Vector2f _vel_ne_filt{}; ///< Stores vel ne filt.

	float _vel_d_filt{0.0f};		///< GNSS filtered Down velocity (m/sec)
	uint64_t _time_last_fail_us{0}; ///< Timestamp or duration value in microseconds: time last fail us.
	uint64_t _time_last_pass_us{0}; ///< Timestamp or duration value in microseconds: time last pass us.
	bool _initial_checks_passed{false}; ///< State flag indicating initial checks passed.
	bool _passed{false}; ///< State flag indicating passed.

	struct Params {
		const int32_t &check_mask; ///< Stores check mask.
		const int32_t &ekf2_req_nsats; ///< Stores ekf2 req nsats.
		const float &ekf2_req_pdop; ///< Stores ekf2 req pdop.
		const float &ekf2_req_eph; ///< Stores ekf2 req eph.
		const float &ekf2_req_epv; ///< Stores ekf2 req epv.
		const float &ekf2_req_sacc; ///< Stores ekf2 req sacc.
		const float &ekf2_req_hdrift; ///< Stores ekf2 req hdrift.
		const float &ekf2_req_vdrift; ///< Stores ekf2 req vdrift.
		const int32_t &ekf2_req_fix; ///< Stores ekf2 req fix.
		const float &ekf2_vel_lim; ///< Stores ekf2 vel lim.
		const uint32_t &min_health_time_us; ///< Timestamp or duration value in microseconds: min health time us.
	};

	const Params _params; ///< Stores params.
	const filter_control_status_u &_control_status; ///< Stores control status.
};
}; // namespace estimator

#endif // !EKF_GNSS_CHECKS_H
