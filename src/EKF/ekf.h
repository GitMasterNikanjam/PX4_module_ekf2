/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
 * @file ekf.h
 * @brief Main error-state EKF for attitude, velocity, position, and aiding sensors.
 *
 * The implementation predicts a quaternion-based inertial state and its
 * covariance from IMU delta-angle/delta-velocity samples, then corrects that
 * state with delayed aiding measurements. Optional fusion paths are controlled
 * by `CONFIG_EKF2_*` build features and runtime parameters. The public state
 * accessors expose estimates and diagnostics; input buffering and common
 * controls are inherited from EstimatorInterface.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#ifndef EKF_EKF_H
#define EKF_EKF_H

#include "estimator_interface.h"
#include "aid_source_status.h"

#if defined(CONFIG_EKF2_GNSS)
# include "aid_sources/gnss/gnss_checks.hpp"
# include "yaw_estimator/EKFGSF_yaw.h"
#endif // CONFIG_EKF2_GNSS

#include "bias_estimator/bias_estimator.hpp"
#include "bias_estimator/height_bias_estimator.hpp"
#include "bias_estimator/position_bias_estimator.hpp"

#include <ekf_derivation/generated/state.h>

#include "aid_sources/ZeroGyroUpdate.hpp"
#include "aid_sources/ZeroVelocityUpdate.hpp"

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION)
# include "aid_sources/aux_global_position/aux_global_position.hpp"
#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION

enum class Likelihood { LOW, MEDIUM, HIGH };
class ExternalVisionVel;

/**
 * @brief Error-state Kalman filter implementing the EKF2 navigation estimator.
 *
 * Call init() before adding sensor data. Add timestamped samples through the
 * EstimatorInterface setters and call update() at the configured prediction
 * cadence. update() consumes buffered measurements at the delayed fusion
 * horizon and maintains a separately predicted low-latency output. State
 * vectors use local NED conventions unless the related accessor documents a
 * different frame; see common.h for input sample contracts.
 *
 * The class is a full filter core but does not acquire sensors, schedule
 * itself, publish outputs, or provide thread synchronization.
 */
class Ekf final : public EstimatorInterface
{
public:
	typedef matrix::Vector<float, State::size> VectorState;
	typedef matrix::SquareMatrix<float, State::size> SquareMatrixState;

	/** @brief Construct the EKF and initialize its state to safe defaults. */
	Ekf()
	{
		reset();
	};

	virtual ~Ekf() = default;

	/**
	 * @brief Reset and initialize filter state and its time base.
	 * @param timestamp Initial monotonic timestamp in microseconds.
	 * @return True when the filter is initialized and ready to accept updates.
	 */
	bool init(uint64_t timestamp) override;

	/** @brief Print a concise diagnostic summary through the configured logger. */
	void print_status();

	/**
	 * @brief Run one prediction/fusion cycle using buffered sensor measurements.
	 * @return True when a filter update was performed successfully.
	 */
	bool update();

	/** @brief Return the latest delayed EKF state estimate. */
	const StateSample &state() const { return _state; }

#if defined(CONFIG_EKF2_BAROMETER)
	/** @brief Return barometric-height innovation and fusion diagnostics. */
	const auto &aid_src_baro_hgt() const { return _aid_src_baro_hgt; }
	/** @brief Return status of the pressure-height bias estimator. */
	const BiasEstimator::status &getBaroBiasEstimatorStatus() const { return _baro_b_est.getStatus(); }
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_TERRAIN)
	/** @brief Return whether the terrain state currently has a valid estimate. */
	bool isTerrainEstimateValid() const { return _terrain_valid; }

	/** @brief Return terrain down-position relative to the local origin, in metres. */
	float getTerrainVertPos() const { return _state.terrain + getEkfGlobalOriginAltitude(); };
	/** @brief Return height above ground level in metres. */
	float getHagl() const { return _state.terrain + _gpos.altitude(); }

	/** @brief Return terrain-state variance in square metres. */
	float getTerrainVariance() const { return P(State::terrain.idx, State::terrain.idx); }

#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_RANGE_FINDER)
	/** @brief Return range-height innovation and fusion diagnostics. */
	const auto &aid_src_rng_hgt() const { return _aid_src_rng_hgt; }

	/** @brief Return filtered range-height rate innovation in m/s. */
	float getHaglRateInnov() const { return _rng_consistency_check.getInnov(); }
	/** @brief Return range-height rate innovation variance. */
	float getHaglRateInnovVar() const { return _rng_consistency_check.getInnovVar(); }
	/** @brief Return filtered signed consistency-test ratio. */
	float getHaglRateInnovRatio() const { return _rng_consistency_check.getSignedTestRatioLpf(); }
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	/** @brief Return optical-flow innovation and fusion diagnostics. */
	const auto &aid_src_optical_flow() const { return _aid_src_optical_flow; }

	/** @brief Return latest body-frame flow-derived velocity in m/s. */
	const Vector2f &getFlowVelBody() const { return _flow_vel_body; }
	/** @brief Return latest flow-derived velocity rotated into local NE, in m/s. */
	Vector2f getFlowVelNE() const { return Vector2f(_R_to_earth * Vector3f(getFlowVelBody()(0), getFlowVelBody()(1), 0.f)); }

	/** @brief Return filtered body-frame flow velocity in m/s. */
	const Vector2f &getFilteredFlowVelBody() const { return _flow_vel_body_lpf.getState(); }
	/** @brief Return filtered flow velocity rotated into local NE, in m/s. */
	Vector2f getFilteredFlowVelNE() const { return Vector2f(_R_to_earth * Vector3f(getFilteredFlowVelBody()(0), getFilteredFlowVelBody()(1), 0.f)); }

	/** @brief Return body-rate-compensated optical-flow angular rate. */
	const Vector2f &getFlowCompensated() const { return _flow_rate_compensated; }
	/** @brief Return the latest uncompensated optical-flow angular rate. */
	const Vector2f &getFlowUncompensated() const { return _flow_sample_delayed.flow_rate; }

	/** @brief Return gyro rate associated with the delayed flow observation. */
	const Vector3f getFlowGyro() const { return _flow_sample_delayed.gyro_rate; }
	/** @brief Return estimated optical-flow gyro bias. */
	const Vector3f &getFlowGyroBias() const { return _flow_gyro_bias; }
	/** @brief Return reference body rate used for flow compensation. */
	const Vector3f &getFlowRefBodyRate() const { return _ref_body_rate; }
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)
	/** @brief Return bitmask of auxiliary global-position sources currently fusing. */
	uint8_t getAgpFusingBitmask() const { return _aux_global_position.sourceFusingBitmask(); }
#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION

	/** @brief Return current heading innovation in radians. */
	float getHeadingInnov() const;
	/** @brief Return heading innovation variance in rad^2. */
	float getHeadingInnovVar() const;
	/** @brief Return heading innovation consistency-test ratio. */
	float getHeadingInnovRatio() const;

#if defined(CONFIG_EKF2_DRAG_FUSION)
	/** @brief Return drag-fusion innovation and consistency diagnostics. */
	const auto &aid_src_drag() const { return _aid_src_drag; }
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	/** @brief Return gravity-fusion innovation and consistency diagnostics. */
	const auto &aid_src_gravity() const { return _aid_src_gravity; }
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_WIND)
	/** @brief Return estimated horizontal wind velocity in local NE, in m/s. */
	const Vector2f &getWindVelocity() const { return _state.wind_vel; };
	/** @brief Return wind-velocity variance per axis. */
	Vector2f getWindVelocityVariance() const { return getStateVariance<State::wind_vel>(); }

	/**
	* @brief Resets the wind states to an external observation
	*
	* @param wind_speed The wind speed in m/s
	* @param wind_direction The azimuth (from true north) to where the wind is heading in radians
	* @param wind_speed_accuracy The 1 sigma accuracy of the wind speed estimate in m/s
	* @param wind_direction_accuracy The 1 sigma accuracy of the wind direction estimate in radians
	*/
	void resetWindToExternalObservation(float wind_speed, float wind_direction, float wind_speed_accuracy,
					    float wind_direction_accuracy);
#endif // CONFIG_EKF2_WIND

	/** @brief Return diagonal state covariance for the selected state block. */
	template <const IdxDof &S>
	matrix::Vector<float, S.dof>getStateVariance() const { return P.slice<S.dof, S.dof>(S.idx, S.idx).diag(); } // calling getStateCovariance().diag() uses more flash space

	/** @brief Return covariance submatrix for the selected state block. */
	template <const IdxDof &S>
	matrix::SquareMatrix<float, S.dof>getStateCovariance() const { return P.slice<S.dof, S.dof>(S.idx, S.idx); }

	/** @brief Return the complete EKF state covariance matrix. */
	const matrix::SquareMatrix<float, State::size> &covariances() const { return P; }
	/** @brief Return one covariance element by row and column index. */
	float stateCovariance(unsigned r, unsigned c) const { return P(r, c); }

	/** @brief Return all state variances as the covariance diagonal. */
	matrix::Vector<float, State::size> covariances_diagonal() const { return P.diag(); }

	/** @brief Return attitude rotation variance expressed in body axes, in rad^2. */
	matrix::Vector3f getRotVarBody() const;
	/** @brief Return attitude rotation variance expressed in NED axes, in rad^2. */
	matrix::Vector3f getRotVarNed() const;
	/** @brief Return yaw variance in rad^2. */
	float getYawVar() const;
	/** @brief Return tilt variance in rad^2. */
	float getTiltVariance() const;

	/** @brief Return velocity-state variance per NED axis. */
	Vector3f getVelocityVariance() const { return getStateVariance<State::vel>(); };

	/** @brief Return position-state variance per NED axis. */
	Vector3f getPositionVariance() const { return getStateVariance<State::pos>(); }

	/** @brief Read WGS-84 origin latitude/longitude, altitude, and set time. */
	void getEkfGlobalOrigin(uint64_t &origin_time, double &latitude, double &longitude, float &origin_alt) const;
	/** @brief Check whether latitude and longitude form a valid geodetic position. */
	bool checkLatLonValidity(double latitude, double longitude);
	/** @brief Check whether altitude is within the estimator's valid range. */
	bool checkAltitudeValidity(float altitude);
	/** @brief Set the WGS-84 origin and initialize associated position states/covariances. */
	bool setEkfGlobalOrigin(double latitude, double longitude, float altitude, float hpos_var = NAN, float vpos_var = NAN);
	/** @brief Reset global position states to a new geodetic observation. */
	bool resetGlobalPositionTo(double latitude, double longitude, float altitude, float hpos_var = NAN,
				   float vpos_var = NAN);

	/** @brief Return 1-sigma global horizontal and vertical position uncertainty in metres. */
	void get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv) const;

	/** @brief Return 1-sigma local horizontal and vertical position uncertainty in metres. */
	void get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv) const;

	/** @brief Return 1-sigma horizontal and vertical velocity uncertainty in m/s. */
	void get_ekf_vel_accuracy(float *ekf_evh, float *ekf_evv) const;

	/**
	 * @brief Return motion and height limits needed to stay within sensor constraints.
	 * @param vxy_max Maximum ground-relative horizontal speed (m/s), or NaN if unlimited.
	 * @param vz_max Maximum ground-relative vertical speed (m/s), or NaN if unlimited.
	 * @param hagl_min Minimum height above ground (m), or NaN if not constrained.
	 * @param hagl_max_z Maximum HAGL for vertical control (m), or NaN if not constrained.
	 * @param hagl_max_xy Maximum HAGL for horizontal control (m), or NaN if not constrained.
	 */
	void get_ekf_ctrl_limits(float *vxy_max, float *vz_max, float *hagl_min, float *hagl_max_z, float *hagl_max_xy) const;

	/** @brief Reset estimated gyro bias to zero. */
	void resetGyroBias();
	/** @brief Reset gyro-bias covariance to its configured initial uncertainty. */
	void resetGyroBiasCov();

	/** @brief Reset estimated accelerometer bias to zero. */
	void resetAccelBias();
	/** @brief Reset accelerometer-bias covariance to its configured initial uncertainty. */
	void resetAccelBiasCov();

	/** @brief Return whether the global horizontal position estimate is valid. */
	bool isGlobalHorizontalPositionValid() const
	{
		return _local_origin_lat_lon.isInitialized() && isLocalHorizontalPositionValid();
	}

	/** @brief Return whether global altitude is valid and a local origin exists. */
	bool isGlobalVerticalPositionValid() const
	{
		return PX4_ISFINITE(_local_origin_alt) && isLocalVerticalPositionValid();
	}

	/** @brief Return whether local horizontal position remains within dead-reckoning limits. */
	bool isLocalHorizontalPositionValid() const
	{
		return !_horizontal_deadreckon_time_exceeded;
	}

	/** @brief Return whether local vertical position remains within dead-reckoning limits. */
	bool isLocalVerticalPositionValid() const
	{
		return !_vertical_position_deadreckon_time_exceeded;
	}

	/** @brief Return whether local vertical velocity remains within dead-reckoning limits. */
	bool isLocalVerticalVelocityValid() const
	{
		return !_vertical_velocity_deadreckon_time_exceeded;
	}

	/** @brief Return whether final yaw alignment, including any in-flight mag alignment, is complete. */
	bool isYawFinalAlignComplete() const
	{
#if defined(CONFIG_EKF2_MAGNETOMETER)
		const bool is_using_mag = (_control_status.flags.mag_3D || _control_status.flags.mag_hdg);
		const bool is_mag_alignment_in_flight_complete = is_using_mag
				&& _control_status.flags.mag_aligned_in_flight
				&& ((_time_delayed_us - _flt_mag_align_start_time) > (uint64_t)1e6);
		return _control_status.flags.yaw_align
		       && (is_mag_alignment_in_flight_complete || !is_using_mag);
#else
		return _control_status.flags.yaw_align;
#endif
	}

	// fuse single direct state measurement (eg NED velocity, NED position, mag earth field, etc)
	// constrain_variances must be false when called from inside constrainStateVar to prevent
	// unbounded recursion (constrainStateVariances can call back into this function).
	/**
	 * @brief Fuse one scalar direct state measurement.
	 * @param innov Measurement innovation.
	 * @param innov_var Innovation variance.
	 * @param R Observation variance.
	 * @param state_index Index of the state being directly observed.
	 * @param constrain_variances Whether to apply covariance bounds after fusion.
	 */
	void fuseDirectStateMeasurement(const float innov, const float innov_var, const float R, const int state_index,
					bool constrain_variances = true);

	/** @brief Apply a scalar Kalman measurement update using gain, Jacobian, noise, and innovation. */
	bool measurementUpdate(VectorState &K, const VectorState &H, const float R, const float innovation);

	/** @brief Return estimated body-frame gyro bias in rad/s. */
	const Vector3f &getGyroBias() const { return _state.gyro_bias; } // get the gyroscope bias in rad/s
	/** @brief Return gyro-bias variance per axis. */
	Vector3f getGyroBiasVariance() const { return getStateVariance<State::gyro_bias>(); } // get the gyroscope bias variance in rad/s
	/** @brief Return configured maximum gyro-bias magnitude in rad/s. */
	float getGyroBiasLimit() const { return _params.ekf2_gyr_b_lim; }
	/** @brief Return configured gyro measurement noise in rad/s. */
	float getGyroNoise() const { return _params.ekf2_gyr_noise; }

	/** @brief Return estimated body-frame accelerometer bias in m/s^2. */
	const Vector3f &getAccelBias() const { return _state.accel_bias; } // get the accelerometer bias in m/s**2
	/** @brief Return accelerometer-bias variance per axis. */
	Vector3f getAccelBiasVariance() const { return getStateVariance<State::accel_bias>(); } // get the accelerometer bias variance in m/s**2
	/** @brief Return configured maximum accelerometer-bias magnitude in m/s^2. */
	float getAccelBiasLimit() const { return _params.ekf2_abl_lim; }

#if defined(CONFIG_EKF2_MAGNETOMETER)
	/** @brief Return estimated earth-frame magnetic field in Gauss. */
	const Vector3f &getMagEarthField() const { return _state.mag_I; }

	/** @brief Return estimated body-frame magnetic bias in Gauss. */
	const Vector3f &getMagBias() const { return _state.mag_B; }
	/** @brief Return magnetic-bias variance per axis. */
	Vector3f getMagBiasVariance() const { return getStateVariance<State::mag_B>(); } // get the mag bias variance in Gauss
	/** @brief Return maximum permitted magnetic-bias magnitude in Gauss. */
	float getMagBiasLimit() const { return 0.5f; } // 0.5 Gauss
#endif // CONFIG_EKF2_MAGNETOMETER

	/** @brief Return true if any accelerometer-bias axis is currently inhibited. */
	bool accel_bias_inhibited() const { return _accel_bias_inhibit[0] || _accel_bias_inhibit[1] || _accel_bias_inhibit[2]; }
	/** @brief Return true if any gyro-bias axis is currently inhibited. */
	bool gyro_bias_inhibited() const { return _gyro_bias_inhibit[0] || _gyro_bias_inhibit[1] || _gyro_bias_inhibit[2]; }

	/** @brief Return state reset deltas and reset counters. */
	const auto &state_reset_status() const { return _state_reset_status; }

	// return the amount the local vertical position changed in the last reset and the number of reset events
	/** @brief Return number of local down-position resets. */
	uint8_t get_posD_reset_count() const { return _state_reset_status.reset_count.posD; }
	/** @brief Read the latest down-position reset delta (m) and reset counter. */
	void get_posD_reset(float *delta, uint8_t *counter) const
	{
		*delta = _state_reset_status.posD_change;
		*counter = _state_reset_status.reset_count.posD;
	}

	/** @brief Return number of height-above-ground resets. */
	uint8_t get_hagl_reset_count() const { return _state_reset_status.reset_count.hagl; }
	/** @brief Read the latest height-above-ground reset delta (m) and counter. */
	void get_hagl_reset(float *delta, uint8_t *counter) const
	{
		*delta = _state_reset_status.hagl_change;
		*counter = _state_reset_status.reset_count.hagl;
	}

	// return the amount the local vertical velocity changed in the last reset and the number of reset events
	/** @brief Return number of local down-velocity resets. */
	uint8_t get_velD_reset_count() const { return _state_reset_status.reset_count.velD; }
	/** @brief Read latest down-velocity reset delta (m/s) and reset counter. */
	void get_velD_reset(float *delta, uint8_t *counter) const
	{
		*delta = _state_reset_status.velD_change;
		*counter = _state_reset_status.reset_count.velD;
	}

	// return the amount the local horizontal position changed in the last reset and the number of reset events
	/** @brief Return number of local north/east position resets. */
	uint8_t get_posNE_reset_count() const { return _state_reset_status.reset_count.posNE; }
	/** @brief Copy latest north/east position reset delta (m) and return counter. */
	void get_posNE_reset(float delta[2], uint8_t *counter) const
	{
		_state_reset_status.posNE_change.copyTo(delta);
		*counter = _state_reset_status.reset_count.posNE;
	}

	// return the amount the local horizontal velocity changed in the last reset and the number of reset events
	/** @brief Return number of local north/east velocity resets. */
	uint8_t get_velNE_reset_count() const { return _state_reset_status.reset_count.velNE; }
	/** @brief Copy latest north/east velocity reset delta (m/s) and return counter. */
	void get_velNE_reset(float delta[2], uint8_t *counter) const
	{
		_state_reset_status.velNE_change.copyTo(delta);
		*counter = _state_reset_status.reset_count.velNE;
	}

	// return the amount the quaternion has changed in the last reset and the number of reset events
	/** @brief Return number of attitude quaternion resets. */
	uint8_t get_quat_reset_count() const { return _state_reset_status.reset_count.quat; }
	/** @brief Copy latest quaternion reset delta and return reset counter. */
	void get_quat_reset(float delta_quat[4], uint8_t *counter) const
	{
		_state_reset_status.quat_change.copyTo(delta_quat);
		*counter = _state_reset_status.reset_count.quat;
	}

	/** @brief Return heading innovation consistency-test ratio. */
	float getHeadingInnovationTestRatio() const;

	/** @brief Return horizontal-velocity innovation test ratio. */
	float getHorizontalVelocityInnovationTestRatio() const;
	/** @brief Return vertical-velocity innovation test ratio. */
	float getVerticalVelocityInnovationTestRatio() const;

	/** @brief Return horizontal-position innovation test ratio. */
	float getHorizontalPositionInnovationTestRatio() const;
	/** @brief Return vertical-position innovation test ratio. */
	float getVerticalPositionInnovationTestRatio() const;

	/** @brief Return airspeed innovation test ratio. */
	float getAirspeedInnovationTestRatio() const;
	/** @brief Return synthetic-sideslip innovation test ratio. */
	float getSyntheticSideslipInnovationTestRatio() const;

	/** @brief Return height-above-ground innovation test ratio. */
	float getHeightAboveGroundInnovationTestRatio() const;

	/** @brief Return bitmask describing which state estimates are currently valid. */
	uint16_t get_ekf_soln_status() const;

	/** @brief Return the currently selected height reference sensor. */
	HeightSensor getHeightSensorRef() const { return _height_sensor_ref; }

#if defined(CONFIG_EKF2_AIRSPEED)
	/** @brief Return current airspeed innovation and fusion diagnostics. */
	const auto &aid_src_airspeed() const { return _aid_src_airspeed; }
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	/** @brief Return current synthetic-sideslip innovation and fusion diagnostics. */
	const auto &aid_src_sideslip() const { return _aid_src_sideslip; }
#endif // CONFIG_EKF2_SIDESLIP

	/** @brief Return diagnostics for fallback fake-height aiding. */
	const auto &aid_src_fake_hgt() const { return _aid_src_fake_hgt; }
	/** @brief Return diagnostics for fallback fake-position aiding. */
	const auto &aid_src_fake_pos() const { return _aid_src_fake_pos; }

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	/** @brief Return external-vision height fusion diagnostics. */
	const auto &aid_src_ev_hgt() const { return _aid_src_ev_hgt; }
	/** @brief Return external-vision position fusion diagnostics. */
	const auto &aid_src_ev_pos() const { return _aid_src_ev_pos; }
	/** @brief Return external-vision velocity fusion diagnostics. */
	const auto &aid_src_ev_vel() const { return _aid_src_ev_vel; }
	/** @brief Return external-vision yaw fusion diagnostics. */
	const auto &aid_src_ev_yaw() const { return _aid_src_ev_yaw; }

	/** @brief Return external-vision height-bias estimator diagnostics. */
	const BiasEstimator::status &getEvHgtBiasEstimatorStatus() const { return _ev_hgt_b_est.getStatus(); }
	/** @brief Return horizontal external-vision bias diagnostics for axis index i. */
	const BiasEstimator::status &getEvPosBiasEstimatorStatus(int i) const { return _ev_pos_b_est.getStatus(i); }
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
	/** @brief Set required continuous GNSS healthy duration, in microseconds. */
	void set_min_required_gps_health_time(uint32_t time_us) { _min_gps_health_time_us = time_us; }

	/** @brief Return GNSS quality-check failure status. */
	const GnssChecks::gps_check_fail_status_u &gps_check_fail_status() const { return _gnss_checks.getFailStatus(); }
	/** @brief Return named GNSS quality-check failure flags. */
	const decltype(GnssChecks::gps_check_fail_status_u::flags) &gps_check_fail_status_flags() const { return _gnss_checks.getFailStatus().flags; }

	/** @brief Return whether current GNSS quality checks pass. */
	bool gps_checks_passed() const { return _gnss_checks.passed(); };

	/** @brief Return GNSS height-bias estimator diagnostics. */
	const BiasEstimator::status &getGpsHgtBiasEstimatorStatus() const { return _gps_hgt_b_est.getStatus(); }

	/** @brief Return GNSS altitude fusion diagnostics. */
	const auto &aid_src_gnss_hgt() const { return _aid_src_gnss_hgt; }
	/** @brief Return GNSS horizontal-position fusion diagnostics. */
	const auto &aid_src_gnss_pos() const { return _aid_src_gnss_pos; }
	/** @brief Return GNSS velocity fusion diagnostics. */
	const auto &aid_src_gnss_vel() const { return _aid_src_gnss_vel; }

# if defined(CONFIG_EKF2_GNSS_YAW)
	/** @brief Return GNSS heading fusion diagnostics. */
	const auto &aid_src_gnss_yaw() const { return _aid_src_gnss_yaw; }
# endif // CONFIG_EKF2_GNSS_YAW

	/** @brief Return whether the emergency yaw estimate is available for reset. */
	bool isYawEmergencyEstimateAvailable() const;

	/** @brief Copy composite and per-model EKF-GSF yaw diagnostics when available. */
	bool getDataEKFGSF(float *yaw_composite, float *yaw_variance, float yaw[N_MODELS_EKFGSF],
			   float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF]);

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
	/** @brief Update expected geomagnetic field from the supplied geodetic position. */
	bool updateWorldMagneticModel(const double latitude_deg, const double longitude_deg);

	/** @brief Return magnetometer innovation and fusion diagnostics. */
	const auto &aid_src_mag() const { return _aid_src_mag; }
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_AUXVEL)
	/** @brief Return auxiliary-velocity fusion diagnostics. */
	const auto &aid_src_aux_vel() const { return _aid_src_aux_vel; }
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_RANGING_BEACON)
	/** @brief Return ranging-beacon fusion diagnostics. */
	const auto &aid_src_ranging_beacon() const { return _aid_src_ranging_beacon; }
#endif // CONFIG_EKF2_RANGING_BEACON

	/** @brief Reset global position to an external geodetic observation and uncertainty. */
	bool resetGlobalPosToExternalObservation(double latitude, double longitude, float altitude, float eph, float epv,
			uint64_t timestamp_observation);

	/** @brief Reset yaw to an external heading measurement and mark heading observable. */
	void resetHeadingToExternalObservation(float heading, float heading_accuracy)
	{
		// External heading is an observation of yaw. Setting heading_observable = true
		// prevents clearInhibitedStateKalmanGains() from inhibiting the update
		_control_status.flags.heading_observable = true;
		const float heading_variance = sq(heading_accuracy);

		if (_control_status.flags.yaw_align) {
			resetYawByFusion(heading, heading_variance);

		} else {
			resetQuatStateYaw(heading, heading_variance);
			_control_status.flags.yaw_align = true;
		}

		// Force the mag consistency check to pass again since an external heading reset is often done to
		// counter mag disturbances.
		_control_status.flags.mag_heading_consistent = false;
		_control_status.flags.yaw_manual = true;
	}

	/** @brief Refresh estimator parameters and apply any required state updates. */
	void updateParameters();

	friend class AuxGlobalPosition;
	friend class AgpSource;

private:

	friend class ExternalVisionVel;
	friend class EvVelBodyFrameFrd;
	friend class EvVelLocalFrameNed;
	friend class EvVelLocalFrameFrd;

	// set the internal states and status to their default value
	/** @brief Restore EKF states, covariance, buffers, and status flags to defaults. */
	void reset();

	/** @brief Initialise tilt. */
	bool initialiseTilt();

	// check if the EKF is dead reckoning horizontal velocity using inertial data only
	/** @brief Update dead reckoning status. */
	void updateDeadReckoningStatus();
	/** @brief Update horizontal dead reckoningstatus. */
	void updateHorizontalDeadReckoningstatus();
	/** @brief Update vertical dead reckoning status. */
	void updateVerticalDeadReckoningStatus();

	static constexpr float kGyroBiasVarianceMin{1e-9f}; ///< Compile-time constant defining kGyroBiasVarianceMin.
	static constexpr float kAccelBiasVarianceMin{1e-9f}; ///< Compile-time constant defining kAccelBiasVarianceMin.

#if defined(CONFIG_EKF2_MAGNETOMETER)
	static constexpr float kMagVarianceMin = 1e-6f; ///< Compile-time constant defining kMagVarianceMin.
#endif // CONFIG_EKF2_MAGNETOMETER


	struct StateResetCounts {
		uint8_t velNE{0};	///< number of horizontal position reset events (allow to wrap if count exceeds 255)
		uint8_t velD{0};	///< number of vertical velocity reset events (allow to wrap if count exceeds 255)
		uint8_t posNE{0};	///< number of horizontal position reset events (allow to wrap if count exceeds 255)
		uint8_t posD{0};	///< number of vertical position reset events (allow to wrap if count exceeds 255)
		uint8_t quat{0};	///< number of quaternion reset events (allow to wrap if count exceeds 255)
		uint8_t hagl{0};	///< number of height above ground level reset events (allow to wrap if count exceeds 255)
	};

	struct StateResets {
		Vector2f velNE_change;  ///< North East velocity change due to last reset (m)
		float velD_change;	///< Down velocity change due to last reset (m/sec)
		Vector2f posNE_change;	///< North, East position change due to last reset (m)
		float posD_change;	///< Down position change due to last reset (m)
		Quatf quat_change;	///< quaternion delta due to last reset - multiply pre-reset quaternion by this to get post-reset quaternion
		float hagl_change;	///< Height above ground level change due to last reset (m)

		StateResetCounts reset_count{}; ///< Stores reset count.
	};

	StateResets _state_reset_status{};	///< reset event monitoring structure containing velocity, position, height and yaw reset information
	StateResetCounts _state_reset_count_prev{}; ///< Stores state reset count prev.

	StateSample _state{};		///< state struct of the ekf running at the delayed time horizon

	LatLonAlt _gpos{0.0, 0.0, 0.f}; ///< Stores gpos.

	bool _filter_initialised{false};	///< true when the EKF sttes and covariances been initialised

	uint64_t _time_last_horizontal_aiding{0}; ///< amount of time we have been doing inertial only deadreckoning (uSec)
	uint64_t _time_last_v_pos_aiding{0}; ///< Timestamp or duration value in microseconds: time last v pos aiding.
	uint64_t _time_last_v_vel_aiding{0}; ///< Timestamp or duration value in microseconds: time last v vel aiding.

	uint64_t _time_last_hor_pos_fuse{0};	///< time the last fusion of horizontal position measurements was performed (uSec)
	uint64_t _time_last_hgt_fuse{0};	///< time the last fusion of vertical position measurements was performed (uSec)
	uint64_t _time_last_hor_vel_fuse{0};	///< time the last fusion of horizontal velocity measurements was performed (uSec)
	uint64_t _time_last_ver_vel_fuse{0};	///< time the last fusion of verticalvelocity measurements was performed (uSec)
	uint64_t _time_last_heading_fuse{0}; ///< Timestamp or duration value in microseconds: time last heading fuse.
	uint64_t _time_last_terrain_fuse{0}; ///< Timestamp or duration value in microseconds: time last terrain fuse.

	LatLonAlt _last_known_gpos{}; ///< Stores last known gpos.

	Vector3f _earth_rate_NED{}; ///< earth rotation vector (NED) in rad/s
	double _earth_rate_lat_ref_rad{0.0}; ///< latitude at which the earth rate was evaluated (radians)

	Dcmf _R_to_earth{};	///< transformation matrix from body frame to earth frame from last EKF prediction

	static constexpr float _kAccelHorizLpfTimeConstant = 1.f; ///< Compile-time constant defining kAccelHorizLpfTimeConstant.
	AlphaFilter<Vector2f> _accel_horiz_lpf{_kAccelHorizLpfTimeConstant}; ///< Low pass filtered horizontal earth frame acceleration (m/sec**2)

#if defined(CONFIG_EKF2_WIND)
	static constexpr float _kHeightRateLpfTimeConstant = 10.f; ///< Compile-time constant defining kHeightRateLpfTimeConstant.
	AlphaFilter<float> _height_rate_lpf{_kHeightRateLpfTimeConstant}; ///< Stores height rate lpf.
#endif // CONFIG_EKF2_WIND

	SquareMatrixState P{};	///< state covariance matrix

#if defined(CONFIG_EKF2_DRAG_FUSION)
	ekf2::AidSourceStatus2D _aid_src_drag {}; ///< Innovation and fusion status for drag.
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_TERRAIN)
	// Terrain height state estimation
	float _last_on_ground_posD{0.0f};	///< last vertical position when the in_air status was false (m)

	bool _terrain_valid{false}; ///< State flag indicating terrain valid.
#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_RANGE_FINDER)
	ekf2::AidSourceStatus1D _aid_src_rng_hgt {}; ///< Innovation and fusion status for rng hgt.
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	ekf2::AidSourceStatus2D _aid_src_optical_flow {}; ///< Innovation and fusion status for optical flow.

	// optical flow processing
	Vector3f _flow_gyro_bias{};	///< bias errors in optical flow sensor rate gyro outputs (rad/sec)
	Vector3f _ref_body_rate{}; ///< Stores ref body rate.

	Vector2f _flow_vel_body{};                      ///< velocity from corrected flow measurement (body frame)(m/s)
	AlphaFilter<Vector2f> _flow_vel_body_lpf{_dt_ekf_avg, _kSensorLpfTimeConstant}; ///< filtered velocity from corrected flow measurement (body frame)(m/s)
	uint32_t _flow_counter{0};                      ///< number of flow samples read for initialization

	Vector2f _flow_rate_compensated{}; ///< measured angular rate of the image about the X and Y body axes after removal of body rotation (rad/s), RH rotation is positive
	AlphaFilter<Vector2f> _flow_rate_compensated_lpf{_dt_ekf_avg, _kSensorLpfTimeConstant}; ///< Stores flow rate compensated lpf.
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_AIRSPEED)
	ekf2::AidSourceStatus1D _aid_src_airspeed {}; ///< Innovation and fusion status for airspeed.
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_SIDESLIP)
	ekf2::AidSourceStatus1D _aid_src_sideslip {}; ///< Innovation and fusion status for sideslip.
#endif // CONFIG_EKF2_SIDESLIP

	ekf2::AidSourceStatus2D _aid_src_fake_pos{}; ///< Innovation and fusion status for fake pos.
	ekf2::AidSourceStatus1D _aid_src_fake_hgt{}; ///< Innovation and fusion status for fake hgt.

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	ekf2::AidSourceStatus1D _aid_src_ev_hgt {}; ///< Innovation and fusion status for ev hgt.
	ekf2::AidSourceStatus2D _aid_src_ev_pos{}; ///< Innovation and fusion status for ev pos.
	ekf2::AidSourceStatus3D _aid_src_ev_vel{}; ///< Innovation and fusion status for ev vel.
	ekf2::AidSourceStatus1D _aid_src_ev_yaw{}; ///< Innovation and fusion status for ev yaw.

	uint8_t _nb_ev_pos_reset_available{0}; ///< Stores nb ev pos reset available.
	uint8_t _nb_ev_vel_reset_available{0}; ///< Stores nb ev vel reset available.
	uint8_t _nb_ev_yaw_reset_available{0}; ///< Stores nb ev yaw reset available.
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
	bool _gps_data_ready {false};	///< true when new GPS data has fallen behind the fusion time horizon and is available to be fused

	// height sensor status
	bool _gps_intermittent{true};           ///< true if data into the buffer is intermittent

	HeightBiasEstimator _gps_hgt_b_est{HeightSensor::GNSS, _height_sensor_ref}; ///< Stores gps hgt b est.

	ekf2::AidSourceStatus1D _aid_src_gnss_hgt{}; ///< Innovation and fusion status for gnss hgt.
	ekf2::AidSourceStatus2D _aid_src_gnss_pos{}; ///< Innovation and fusion status for gnss pos.
	ekf2::AidSourceStatus3D _aid_src_gnss_vel{}; ///< Innovation and fusion status for gnss vel.

	uint64_t _time_last_gnss_hgt_rejected{0}; ///< Timestamp or duration value in microseconds: time last gnss hgt rejected.

# if defined(CONFIG_EKF2_GNSS_YAW)
	ekf2::AidSourceStatus1D _aid_src_gnss_yaw {}; ///< Innovation and fusion status for gnss yaw.
# endif // CONFIG_EKF2_GNSS_YAW
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_RANGING_BEACON)
	ekf2::AidSourceStatus1D _aid_src_ranging_beacon {}; ///< Innovation and fusion status for ranging beacon.
#endif // CONFIG_EKF2_RANGING_BEACON

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	ekf2::AidSourceStatus3D _aid_src_gravity {}; ///< Innovation and fusion status for gravity.
#endif // CONFIG_EKF2_GRAVITY_FUSION

#if defined(CONFIG_EKF2_AUXVEL)
	ekf2::AidSourceStatus2D _aid_src_aux_vel {}; ///< Innovation and fusion status for aux vel.
#endif // CONFIG_EKF2_AUXVEL

	// Variables used by the initial filter alignment
	bool _is_first_imu_sample{true}; ///< State flag indicating is first imu sample.
	static constexpr float _kSensorLpfTimeConstant = 0.09f; ///< Compile-time constant defining kSensorLpfTimeConstant.
	AlphaFilter<Vector3f> _accel_lpf{_dt_ekf_avg, _kSensorLpfTimeConstant};	///< filtered accelerometer measurement used to align tilt (m/s/s)
	AlphaFilter<Vector3f> _gyro_lpf{_dt_ekf_avg, _kSensorLpfTimeConstant};	///< filtered gyro measurement used for alignment excessive movement check (rad/sec)

#if defined(CONFIG_EKF2_BAROMETER)
	ekf2::AidSourceStatus1D _aid_src_baro_hgt {}; ///< Innovation and fusion status for baro hgt.

	// Variables used to perform in flight resets and switch between height sources
	AlphaFilter<float> _baro_lpf{_dt_ekf_avg, _kSensorLpfTimeConstant};	///< filtered barometric height measurement (m)
	uint32_t _baro_counter{0};		///< number of baro samples read during initialisation

	HeightBiasEstimator _baro_b_est{HeightSensor::BARO, _height_sensor_ref}; ///< Stores baro b est.

#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// used by magnetometer fusion mode selection
	AlphaFilter<float> _mag_heading_innov_lpf{_dt_ekf_avg, _kSensorLpfTimeConstant}; ///< Stores mag heading innov lpf.
	uint32_t _min_mag_health_time_us{1'000'000}; ///< magnetometer is marked as healthy only after this amount of time

	ekf2::AidSourceStatus3D _aid_src_mag{}; ///< Innovation and fusion status for mag.

	AlphaFilter<Vector3f> _mag_lpf{_dt_ekf_avg, _kSensorLpfTimeConstant};	///< filtered magnetometer measurement for instant reset (Gauss)
	uint32_t _mag_counter{0};		///< number of magnetometer samples read during initialisation

	// Variables used to control activation of post takeoff functionality
	uint64_t _flt_mag_align_start_time{0};	///< time that inflight magnetic field alignment started (uSec)
	uint64_t _time_last_mag_check_failing{0}; ///< Timestamp or duration value in microseconds: time last mag check failing.
#endif // CONFIG_EKF2_MAGNETOMETER

	// variables used to inhibit accel bias learning
	bool _accel_bias_inhibit[3] {};		///< true when the accel bias learning is being inhibited for the specified axis
	bool _gyro_bias_inhibit[3] {};		///< true when the gyro bias learning is being inhibited for the specified axis
	float _accel_magnitude_filt{0.0f};	///< acceleration magnitude after application of a decaying envelope filter (rad/sec)
	float _ang_rate_magnitude_filt{0.0f};		///< angular rate magnitude after application of a decaying envelope filter (rad/sec)

	// imu fault status
	uint64_t _time_bad_vert_accel{0};	///< last time a bad vertical accel was detected (uSec)
	uint64_t _time_good_vert_accel{0};	///< last time a good vertical accel was detected (uSec)
	uint16_t _clip_counter[3];		///< counter per axis that increments when clipping ad decrements when not

	// initialise filter states of both the delayed ekf and the real time complementary filter
	/** @brief Initialise filter. */
	bool initialiseFilter(void);

	// initialise ekf covariance matrix
	/** @brief Initialise covariance. */
	void initialiseCovariance();

	// predict ekf state
	/** @brief Predict state. */
	void predictState(const imuSample &imu_delayed);

	// predict ekf covariance
	/** @brief Predict covariance. */
	void predictCovariance(const imuSample &imu_delayed);

	/** @brief Reset covariance block for state S using the supplied covariance matrix. */
	template <const IdxDof &S>
	void resetStateCovariance(const matrix::SquareMatrix<float, S.dof> &cov)
	{
		P.uncorrelateCovarianceSetVariance<S.dof>(S.idx, 0.0f);
		P.slice<S.dof, S.dof>(S.idx, S.idx) = cov;
	}

	bool setLatLonOrigin(double latitude, double longitude, float hpos_var = NAN);
	bool setAltOrigin(float altitude, float vpos_var = NAN);

	bool resetLatLonTo(double latitude, double longitude, float hpos_var = NAN);
	bool initialiseAltitudeTo(float altitude, float vpos_var = NAN);

	// update quaternion states and covariances using an innovation, observation variance and Jacobian vector
	bool fuseYaw(ekf2::AidSourceStatus1D &aid_src_status, const VectorState &H_YAW, const bool reset = false);
	/** @brief Compute yaw innov var and h. */
	void computeYawInnovVarAndH(float observation_variance, float &innovation_variance, VectorState &H_YAW) const;

	/** @brief Update imubias inhibit. */
	void updateIMUBiasInhibit(const imuSample &imu_delayed);

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// ekf sequential fusion of magnetometer measurements
	/** @brief Fuse mag. */
	bool fuseMag(const Vector3f &mag, const float R_MAG, VectorState &H, ekf2::AidSourceStatus3D &aid_src,
		     bool update_all_states = false, bool update_tilt = false); ///< State flag indicating update all states.

	// fuse magnetometer declination measurement
	//  R: declination observation variance (rad**2)
	bool fuseDeclination(const float decl_measurement_rad, const float R, bool update_all_states = false,
			     bool update_tilt = false); ///< State flag indicating update tilt.

#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_AIRSPEED)
	// control fusion of air data observations
	/** @brief Control air data fusion. */
	void controlAirDataFusion(const imuSample &imu_delayed);

	/** @brief Update airspeed. */
	void updateAirspeed(const airspeedSample &airspeed_sample, ekf2::AidSourceStatus1D &aid_src) const;
	/** @brief Fuse airspeed. */
	void fuseAirspeed(const airspeedSample &airspeed_sample, ekf2::AidSourceStatus1D &aid_src);

	/** @brief Stop airspeed fusion. */
	void stopAirspeedFusion();

	// Reset the wind states using the current airspeed measurement, ground relative nav velocity, yaw angle and assumption of zero sideslip
	/** @brief Reset wind using airspeed. */
	void resetWindUsingAirspeed(const airspeedSample &airspeed_sample);
	/** @brief Reset vel using airspeed. */
	void resetVelUsingAirspeed(const airspeedSample &airspeed_sample);
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	// control fusion of synthetic sideslip observations
	/** @brief Control beta fusion. */
	void controlBetaFusion(const imuSample &imu_delayed);

	// fuse synthetic zero sideslip measurement
	/** @brief Update sideslip. */
	void updateSideslip(ekf2::AidSourceStatus1D &_aid_src_sideslip) const;
	/** @brief Fuse sideslip. */
	bool fuseSideslip(ekf2::AidSourceStatus1D &_aid_src_sideslip);
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// control fusion of multi-rotor drag specific force observations
	/** @brief Control drag fusion. */
	void controlDragFusion(const imuSample &imu_delayed);

	// fuse body frame drag specific forces for multi-rotor wind estimation
	/** @brief Fuse drag. */
	void fuseDrag(const dragSample &drag_sample);
#endif // CONFIG_EKF2_DRAG_FUSION

	/** @brief Reset velocity to. */
	void resetVelocityTo(const Vector3f &vel, const Vector3f &new_vel_var);

	/** @brief Reset horizontal velocity to. */
	void resetHorizontalVelocityTo(const Vector2f &new_horz_vel, const Vector2f &new_horz_vel_var);
	/** @brief Reset horizontal velocity to. */
	void resetHorizontalVelocityTo(const Vector2f &new_horz_vel, float vel_var) { resetHorizontalVelocityTo(new_horz_vel, Vector2f(vel_var, vel_var)); }

	/** @brief Reset horizontal velocity to zero. */
	void resetHorizontalVelocityToZero();

	/** @brief Reset vertical velocity to. */
	void resetVerticalVelocityTo(float new_vert_vel, float new_vert_vel_var);


	/** @brief Reset horizontal position to last known. */
	void resetHorizontalPositionToLastKnown();

	/** @brief Reset horizontal position to. */
	void resetHorizontalPositionTo(const double &new_latitude, const double &new_longitude,
				       const Vector2f &new_horz_pos_var);
	void resetHorizontalPositionTo(const double &new_latitude, const double &new_longitude, const float pos_var = NAN) { resetHorizontalPositionTo(new_latitude, new_longitude, Vector2f(pos_var, pos_var)); }
	/** @brief Reset horizontal position to. */
	void resetHorizontalPositionTo(const Vector2f &new_pos, const Vector2f &new_horz_pos_var);

	/** @brief Get local horizontal position. */
	Vector2f getLocalHorizontalPosition() const;

	/** @brief Compute delta horizontal position. */
	Vector2f computeDeltaHorizontalPosition(const double &new_latitude, const double &new_longitude) const;
	/** @brief Update horizontal position reset status. */
	void updateHorizontalPositionResetStatus(const Vector2f &delta);

	/** @brief Reset wind to. */
	void resetWindTo(const Vector2f &wind, const Vector2f &wind_var);

	/** @brief Is height reset required. */
	bool isHeightResetRequired() const;

	void resetAltitudeTo(float new_altitude, float new_vert_pos_var = NAN);
	/** @brief Update vertical position reset status. */
	void updateVerticalPositionResetStatus(const float delta_z);

	/** @brief Reset vertical velocity to zero. */
	void resetVerticalVelocityToZero();

	// horizontal and vertical position aid source
	/** @brief Update vertical position aid status. */
	void updateVerticalPositionAidStatus(ekf2::AidSourceStatus1D &aid_src, const uint64_t &time_us,
					     const float observation, const float observation_variance, const float innovation_gate = 1.f) const;

	// horizontal and vertical position fusion
	/** @brief Fuse horizontal position. */
	bool fuseHorizontalPosition(ekf2::AidSourceStatus2D &pos_aid_src);
	/** @brief Fuse vertical position. */
	bool fuseVerticalPosition(ekf2::AidSourceStatus1D &hgt_aid_src);

	// 2d & 3d velocity fusion
	/** @brief Fuse horizontal velocity. */
	bool fuseHorizontalVelocity(ekf2::AidSourceStatus2D &vel_aid_src);
	/** @brief Fuse velocity. */
	bool fuseVelocity(ekf2::AidSourceStatus3D &vel_aid_src);

#if defined(CONFIG_EKF2_TERRAIN)
	/** @brief Init terrain. */
	void initTerrain();
	/** @brief Get terrain vpos. */
	float getTerrainVPos() const { return isTerrainEstimateValid() ? _state.terrain : _last_on_ground_posD; }
	/** @brief Control terrain fake fusion. */
	void controlTerrainFakeFusion();

	/** @brief Update terrain validity. */
	void updateTerrainValidity();
	/** @brief Update terrain reset status. */
	void updateTerrainResetStatus(const float delta_z);

# if defined(CONFIG_EKF2_RANGE_FINDER)
	// update the terrain vertical position estimate using a height above ground measurement from the range finder
	/** @brief Fuse hagl rng. */
	bool fuseHaglRng(ekf2::AidSourceStatus1D &aid_src, bool update_height, bool update_terrain);
	/** @brief Reset terrain to rng. */
	void resetTerrainToRng(ekf2::AidSourceStatus1D &aid_src);
	/** @brief Get rng var. */
	float getRngVar() const;
# endif // CONFIG_EKF2_RANGE_FINDER

# if defined(CONFIG_EKF2_OPTICAL_FLOW)
	/** @brief Reset terrain to flow. */
	void resetTerrainToFlow();
# endif // CONFIG_EKF2_OPTICAL_FLOW

#endif // CONFIG_EKF2_TERRAIN

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// range height
	/** @brief Control range hagl fusion. */
	void controlRangeHaglFusion(const imuSample &imu_delayed);
	/** @brief Is conditional range aid suitable. */
	bool isConditionalRangeAidSuitable();
	/** @brief Stop rng hgt fusion. */
	void stopRngHgtFusion();
	/** @brief Stop rng terr fusion. */
	void stopRngTerrFusion();
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// control fusion of optical flow observations
	/** @brief Control optical flow fusion. */
	void controlOpticalFlowFusion(const imuSample &imu_delayed);
	/** @brief Reset flow fusion. */
	void resetFlowFusion(const flowSample &flow_sample);
	/** @brief Stop flow fusion. */
	void stopFlowFusion();

	/** @brief Update on ground motion for optical flow checks. */
	void updateOnGroundMotionForOpticalFlowChecks();
	/** @brief Reset on ground motion for optical flow checks. */
	void resetOnGroundMotionForOpticalFlowChecks();

	// calculate the measurement variance for the optical flow sensor (rad/sec)^2
	/** @brief Calculate optical-flow observation variance from quality and range. */
	float calcOptFlowMeasVar(const flowSample &flow_sample) const;

	// calculate optical flow body angular rate compensation
	/** @brief Calculate body-rate compensation for the latest optical-flow sample. */
	void calcOptFlowBodyRateComp(const flowSample &flow_sample);

	/** @brief Predict flow hagl. */
	float predictFlowHagl() const;
	/** @brief Predict flow range. */
	float predictFlowRange() const;
	/** @brief Predict flow. */
	Vector2f predictFlow(const Vector3f &flow_gyro) const;

	// fuse optical flow line of sight rate measurements
	/** @brief Fuse opt flow. */
	bool fuseOptFlow(VectorState &H, bool update_terrain);

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// Return the magnetic declination in radians to be used by the alignment and fusion processing
	/** @brief Get mag declination. */
	float getMagDeclination();
#endif // CONFIG_EKF2_MAGNETOMETER

	/** @brief Clear inhibited state kalman gains. */
	void clearInhibitedStateKalmanGains(VectorState &K) const;

	// limit the diagonal of the covariance matrix
	/** @brief Constrain state variances. */
	void constrainStateVariances();

	/** @brief Constrain state var. */
	void constrainStateVar(const IdxDof &state, float min, float max);
	void constrainStateVarLimitRatio(const IdxDof &state, float min, float max, float max_ratio = 1.e6f);

	/** @brief Remove heading cross-correlation and constrain heading covariance. */
	void uncorrelateAndLimitHeadingCovariance();

	// generic function which will perform a fusion step given a kalman gain K
	// and a scalar innovation value
	/** @brief Apply a Kalman correction vector to the state using one innovation. */
	void fuse(const VectorState &K, float innovation);

	// calculate the earth rotation vector from a given latitude
	/** @brief Calculate Earth rotation vector in NED coordinates at latitude. */
	Vector3f calcEarthRateNED(float lat_rad) const;

	// Control the filter fusion modes
	/** @brief Control fusion modes. */
	void controlFusionModes(const imuSample &imu_delayed);

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// control fusion of external vision observations
	/** @brief Control external vision fusion. */
	void controlExternalVisionFusion(const imuSample &imu_sample);
	/** @brief Update ev attitude error filter. */
	void updateEvAttitudeErrorFilter(extVisionSample &ev_sample, bool ev_reset);
	/** @brief Control ev height fusion. */
	void controlEvHeightFusion(const imuSample &imu_sample, const extVisionSample &ev_sample,
				   const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient,
				   ekf2::AidSourceStatus1D &aid_src);
	/** @brief Control ev pos fusion. */
	void controlEvPosFusion(const imuSample &imu_sample, const extVisionSample &ev_sample,
				const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient,
				ekf2::AidSourceStatus2D &aid_src);
	/** @brief Control ev vel fusion. */
	void controlEvVelFusion(ExternalVisionVel &ev, const bool common_starting_conditions_passing, const bool ev_reset,
				const bool quality_sufficient, ekf2::AidSourceStatus3D &aid_src);
	/** @brief Control ev yaw fusion. */
	void controlEvYawFusion(const imuSample &imu_sample, const extVisionSample &ev_sample,
				const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient,
				ekf2::AidSourceStatus1D &aid_src);
	/** @brief Fuse local frame velocity. */
	void fuseLocalFrameVelocity(ekf2::AidSourceStatus3D &aid_src, const uint64_t &timestamp, const Vector3f &measurement,
				    const Vector3f &measurement_var, const float &innovation_gate);
	/** @brief Fuse body frame velocity. */
	void fuseBodyFrameVelocity(ekf2::AidSourceStatus3D &aid_src, const uint64_t &timestamp, const Vector3f &measurement,
				   const Vector3f &measurement_var, const float &innovation_gate);

	/** @brief Start ev pos fusion. */
	void startEvPosFusion(const Vector2f &measurement, const Vector2f &measurement_var, ekf2::AidSourceStatus2D &aid_src);
	/** @brief Update ev pos fusion. */
	void updateEvPosFusion(const Vector2f &measurement, const Vector2f &measurement_var, bool quality_sufficient,
			       bool reset, ekf2::AidSourceStatus2D &aid_src);

	/** @brief Stop ev pos fusion. */
	void stopEvPosFusion();
	/** @brief Stop ev hgt fusion. */
	void stopEvHgtFusion();
	/** @brief Stop ev vel fusion. */
	void stopEvVelFusion();
	/** @brief Stop ev yaw fusion. */
	void stopEvYawFusion();
	/** @brief Fuse ev velocity. */
	bool fuseEvVelocity(ekf2::AidSourceStatus3D &aid_src, const extVisionSample &ev_sample);
	/** @brief Fuse body velocity. */
	void fuseBodyVelocity(ekf2::AidSourceStatus1D &aid_src, float &innov_var, VectorState &H)
	{
		VectorState Kfusion = P * H / innov_var; ///< Stores Kfusion.
		measurementUpdate(Kfusion, H, aid_src.observation_variance, aid_src.innovation);
		aid_src.fused = true;
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_GNSS)
	// control fusion of GPS observations
	/** @brief Control gps fusion. */
	void controlGpsFusion(const imuSample &imu_delayed);
	/** @brief Control gnss vel fusion. */
	void controlGnssVelFusion(ekf2::AidSourceStatus3D &aid_src, bool force_reset);
	/** @brief Control gnss pos fusion. */
	void controlGnssPosFusion(ekf2::AidSourceStatus2D &aid_src, const bool force_reset);
	/** @brief Stop gnss fusion. */
	void stopGnssFusion();
	/** @brief Stop gnss vel fusion. */
	void stopGnssVelFusion();
	/** @brief Stop gnss pos fusion. */
	void stopGnssPosFusion();
	/** @brief Update gnss vel. */
	void updateGnssVel(const imuSample &imu_sample, const gnssSample &gnss_sample, ekf2::AidSourceStatus3D &aid_src);
	/** @brief Update gnss pos. */
	void updateGnssPos(const gnssSample &gnss_sample, ekf2::AidSourceStatus2D &aid_src);
	/** @brief Is gnss vel reset allowed. */
	bool isGnssVelResetAllowed() const;
	/** @brief Is gnss pos reset allowed. */
	bool isGnssPosResetAllowed() const;
	/** @brief Control gnss yaw estimator. */
	void controlGnssYawEstimator(ekf2::AidSourceStatus3D &aid_src_vel);
	/** @brief Try yaw emergency reset. */
	bool tryYawEmergencyReset();
	/** @brief Reset velocity to gnss. */
	void resetVelocityToGnss(ekf2::AidSourceStatus3D &aid_src);
	/** @brief Reset horizontal position to gnss. */
	void resetHorizontalPositionToGnss(ekf2::AidSourceStatus2D &aid_src);

	/** @brief Control gnss height fusion. */
	void controlGnssHeightFusion(const gnssSample &gps_sample);
	/** @brief Stop gps hgt fusion. */
	void stopGpsHgtFusion();
	/** @brief Is gnss hgt reset allowed. */
	bool isGnssHgtResetAllowed();

# if defined(CONFIG_EKF2_GNSS_YAW)
	/** @brief Control gnss yaw fusion. */
	void controlGnssYawFusion(const gnssSample &gps_sample);
	/** @brief Stop gnss yaw fusion. */
	void stopGnssYawFusion();

	// fuse the yaw angle obtained from a dual antenna GPS unit
	/** @brief Fuse gnss yaw. */
	void fuseGnssYaw(float antenna_yaw_offset);

	// reset the quaternions states using the yaw angle obtained from a dual antenna GPS unit
	// return true if the reset was successful
	/** @brief Reset yaw to gnss. */
	bool resetYawToGnss(float gnss_yaw, float gnss_yaw_offset);

	/** @brief Update gnss yaw. */
	void updateGnssYaw(const gnssSample &gps_sample);

# endif // CONFIG_EKF2_GNSS_YAW

	// Declarations used to control use of the EKF-GSF yaw estimator
	/** @brief Is yaw failure. */
	bool isYawFailure() const;

	// Resets the main Nav EKf yaw to the estimator from the EKF-GSF yaw estimator
	// Returns true if the reset was successful
	/** @brief Reset yaw to ekfgsf. */
	bool resetYawToEKFGSF();

	// yaw estimator instance
	EKFGSF_yaw _yawEstimator{}; ///< Stores yawEstimator.

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_RANGING_BEACON)
	/** @brief Control ranging beacon fusion. */
	void controlRangingBeaconFusion(const imuSample &imu_delayed);
	/** @brief Fuse ranging beacon. */
	void fuseRangingBeacon(const rangingBeaconSample &sample);
	/** @brief Stop ranging beacon fusion. */
	void stopRangingBeaconFusion();
#endif // CONFIG_EKF2_RANGING_BEACON

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// control fusion of magnetometer observations
	/** @brief Control mag fusion. */
	void controlMagFusion(const imuSample &imu_sample);

	/** @brief Check hagl yaw reset req. */
	bool checkHaglYawResetReq() const;

	/** @brief Reset mag heading. */
	void resetMagHeading(const Vector3f &mag);
	void resetMagStates(const Vector3f &mag, bool reset_heading = true);
	/** @brief Check whether range-derived HAGL conditions request a yaw reset. */
	bool haglYawResetReq();

	/** @brief Check mag heading consistency. */
	void checkMagHeadingConsistency(const magSample &mag_sample);

	/** @brief Check mag field. */
	bool checkMagField(const Vector3f &mag);
	/** @brief Is measured matching expected. */
	static bool isMeasuredMatchingExpected(float measured, float expected, float gate);

	/** @brief Stop mag fusion. */
	void stopMagFusion();

	// calculate a synthetic value for the magnetometer Z component, given the 3D magnetomter
	// sensor measurement
	/** @brief Calculate synthetic mag z measurement. */
	float calculate_synthetic_mag_z_measurement(const Vector3f &mag_meas, const Vector3f &mag_earth_predicted);

#endif // CONFIG_EKF2_MAGNETOMETER

	// control fusion of fake position observations to constrain drift
	/** @brief Control fake pos fusion. */
	void controlFakePosFusion();

	/** @brief Control fake hgt fusion. */
	void controlFakeHgtFusion();
	/** @brief Reset fake hgt fusion. */
	void resetFakeHgtFusion();
	/** @brief Reset height to last known. */
	void resetHeightToLastKnown();
	/** @brief Stop fake hgt fusion. */
	void stopFakeHgtFusion();

#if defined(CONFIG_EKF2_AUXVEL)
	// control fusion of auxiliary velocity observations
	/** @brief Control aux vel fusion. */
	void controlAuxVelFusion(const imuSample &imu_sample);
	/** @brief Stop aux vel fusion. */
	void stopAuxVelFusion();
#endif // CONFIG_EKF2_AUXVEL

	/** @brief Check vertical acceleration health. */
	void checkVerticalAccelerationHealth(const imuSample &imu_delayed);
	/** @brief Estimate inertial nav falling likelihood. */
	Likelihood estimateInertialNavFallingLikelihood() const;

	// control for combined height fusion mode (implemented for switching between baro and range height)
	/** @brief Control height fusion. */
	void controlHeightFusion(const imuSample &imu_delayed);
	/** @brief Check height sensor ref fallback. */
	void checkHeightSensorRefFallback();

#if defined(CONFIG_EKF2_BAROMETER)
	/** @brief Control baro height fusion. */
	void controlBaroHeightFusion(const imuSample &imu_sample);
	/** @brief Stop baro hgt fusion. */
	void stopBaroHgtFusion();

	/** @brief Update ground effect. */
	void updateGroundEffect();

# if defined(CONFIG_EKF2_BARO_COMPENSATION)
	/** @brief Correct barometric altitude for airspeed-dependent pressure error. */
	float compensateBaroForDynamicPressure(const imuSample &imu_sample, float baro_alt_uncompensated) const;
# endif // CONFIG_EKF2_BARO_COMPENSATION

#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	// gravity fusion: heuristically enable / disable gravity fusion
	/** @brief Control gravity fusion. */
	void controlGravityFusion(const imuSample &imu_delayed);
#endif // CONFIG_EKF2_GRAVITY_FUSION

	void resetQuatCov(const float yaw_noise = NAN);
	/** @brief Reset quat cov. */
	void resetQuatCov(const Vector3f &rot_var_ned);

#if defined(CONFIG_EKF2_MAGNETOMETER)
	/** @brief Reset mag earth cov. */
	void resetMagEarthCov();
	/** @brief Reset mag bias cov. */
	void resetMagBiasCov();
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_WIND)
	/** @brief Reset wind cov. */
	void resetWindCov();
	/** @brief Reset wind to zero. */
	void resetWindToZero();
#endif // CONFIG_EKF2_WIND

	/** @brief Reset gyro bias zcov. */
	void resetGyroBiasZCov();

	/** @brief Is timed out. */
	bool isTimedOut(uint64_t last_sensor_timestamp, uint64_t timeout_period) const
	{
		return (last_sensor_timestamp == 0) || (last_sensor_timestamp + timeout_period < _time_delayed_us);
	}

	/** @brief Is recent. */
	bool isRecent(uint64_t sensor_timestamp, uint64_t acceptance_interval) const
	{
		return (sensor_timestamp != 0) && (sensor_timestamp + acceptance_interval > _time_delayed_us);
	}

	/** @brief Is newest sample recent. */
	bool isNewestSampleRecent(uint64_t sensor_timestamp, uint64_t acceptance_interval) const
	{
		return (sensor_timestamp != 0) && (sensor_timestamp + acceptance_interval > _time_latest_us);
	}

	/** @brief Reset fake pos fusion. */
	void resetFakePosFusion();
	/** @brief Run fake pos state machine. */
	bool runFakePosStateMachine(bool enable_condition_passing, bool status_flag, ekf2::AidSourceStatus2D &aid_src);

	// reset the quaternion states and covariances to the new yaw value, preserving the roll and pitch
	// yaw : Euler yaw angle (rad)
	// yaw_variance : yaw error variance (rad^2)
	/** @brief Reset quat state yaw. */
	void resetQuatStateYaw(float yaw, float yaw_variance);
	/** @brief Propagate quat reset. */
	void propagateQuatReset(const Quatf &quat_before_reset);
	/** @brief Reset yaw by fusion. */
	void resetYawByFusion(float yaw, float yaw_variance);
	/** @brief Reset horizontal velocity to match yaw. */
	void resetHorizontalVelocityToMatchYaw(float delta_yaw);

	HeightSensor _height_sensor_ref{HeightSensor::UNKNOWN}; ///< Stores height sensor ref.
	PositionSensor _position_sensor_ref{PositionSensor::GNSS}; ///< Stores position sensor ref.

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	HeightBiasEstimator _ev_hgt_b_est {HeightSensor::EV, _height_sensor_ref}; ///< Stores ev hgt b est.
	PositionBiasEstimator _ev_pos_b_est{PositionSensor::EV, _position_sensor_ref}; ///< Stores ev pos b est.
	static constexpr float _kQuatErrorLpfTimeConstant = 10.f; ///< Compile-time constant defining kQuatErrorLpfTimeConstant.
	AlphaFilter<Quatf> _ev_q_error_filt{_dt_ekf_avg, _kQuatErrorLpfTimeConstant}; ///< Stores ev q error filt.
	bool _ev_q_error_initialized{false}; ///< State flag indicating ev q error initialized.
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// state was reset to aid source, keep observation and update all other fields appropriately (zero innovation, etc)
	/** @brief Reset aid source status zero innovation. */
	void resetAidSourceStatusZeroInnovation(ekf2::AidSourceStatus1D &status) const;

	// helper used for populating and filtering estimator aid source struct for logging
	/** @brief Update aid source status. */
	void updateAidSourceStatus(ekf2::AidSourceStatus1D &status, const uint64_t &timestamp_sample,
				   const float &observation, const float &observation_variance,
				   const float &innovation, const float &innovation_variance,
				   float innovation_gate = 1.f) const; ///< Stores innovation gate.

	// state was reset to aid source, keep observation and update all other fields appropriately (zero innovation, etc)
	/** @brief Clear innovation fields in an aiding-source status record. */
	template <typename T>
	void resetAidSourceStatusZeroInnovation(T &status) const
	{
		status.time_last_fuse = _time_delayed_us;

		for (size_t i = 0; i < (sizeof(status.observation) / sizeof(status.observation[0])); i++) {
			status.innovation[i] = 0.f;
			status.innovation_filtered[i] = 0.f;
			status.innovation_variance[i] = status.observation_variance[i];

			status.test_ratio[i] = 0.f;
			status.test_ratio_filtered[i] = 0.f;
		}

		status.innovation_rejected = false;
		status.fused = true;
	}

	// helper used for populating and filtering estimator aid source struct for logging
	/** @brief Update aiding-source diagnostics from the current observation and innovation. */
	template <typename T, typename S, typename D>
	void updateAidSourceStatus(T &status, const uint64_t &timestamp_sample,
				   const D &observation, const S &observation_variance,
				   const S &innovation, const S &innovation_variance,
				   float innovation_gate = 1.f) const
	{
		bool innovation_rejected = false; ///< State flag indicating innovation rejected.

		const float dt_s = math::constrain((timestamp_sample - status.timestamp_sample) * 1e-6f, 0.001f, 1.f);

		static constexpr float tau = 0.5f; ///< Compile-time constant defining tau.
		const float alpha = math::constrain(dt_s / (dt_s + tau), 0.f, 1.f);

		for (size_t i = 0; i < (sizeof(status.observation) / sizeof(status.observation[0])); i++) {

			const float test_ratio = sq(innovation(i)) / (sq(innovation_gate) * innovation_variance(i));

			if ((status.timestamp_sample > 0) && (timestamp_sample > status.timestamp_sample)) {

				// test_ratio_filtered
				if (PX4_ISFINITE(status.test_ratio_filtered[i])) {
					status.test_ratio_filtered[i] += alpha * (matrix::sign(innovation(i)) * test_ratio - status.test_ratio_filtered[i]);

				} else {
					// otherwise, init the filtered test ratio
					status.test_ratio_filtered[i] = test_ratio;
				}

				// innovation_filtered
				if (PX4_ISFINITE(status.innovation_filtered[i])) {
					status.innovation_filtered[i] += alpha * (innovation(i) - status.innovation_filtered[i]);

				} else {
					// otherwise, init the filtered innovation
					status.innovation_filtered[i] = innovation(i);
				}

				// limit extremes in filtered values
				static constexpr float kNormalizedInnovationLimit = 2.f; ///< Compile-time constant defining kNormalizedInnovationLimit.
				static constexpr float kTestRatioLimit = sq(kNormalizedInnovationLimit);

				if (test_ratio > kTestRatioLimit) {

					status.test_ratio_filtered[i] = math::constrain(status.test_ratio_filtered[i], -kTestRatioLimit, kTestRatioLimit);

					const float innov_limit = kNormalizedInnovationLimit * innovation_gate * sqrtf(innovation_variance(i));
					status.innovation_filtered[i] = math::constrain(status.innovation_filtered[i], -innov_limit, innov_limit);
				}

			} else {
				// invalid timestamp_sample, reset
				status.test_ratio_filtered[i] = test_ratio;
				status.innovation_filtered[i] = innovation(i);
			}

			status.test_ratio[i] = test_ratio;

			status.observation[i] = static_cast<double>(observation(i));
			status.observation_variance[i] = observation_variance(i);

			status.innovation[i] = innovation(i);
			status.innovation_variance[i] = innovation_variance(i);

			if ((test_ratio > 1.f)
			    || !PX4_ISFINITE(test_ratio)
			    || !PX4_ISFINITE(status.innovation[i])
			    || !PX4_ISFINITE(status.innovation_variance[i])
			   ) {
				innovation_rejected = true;
			}
		}

		status.timestamp_sample = timestamp_sample;

		// if any of the innovations are rejected, then the overall innovation is rejected
		status.innovation_rejected = innovation_rejected;

		// reset
		status.fused = false;
	}

	ZeroGyroUpdate _zero_gyro_update{}; ///< Stores zero gyro update.
	ZeroVelocityUpdate _zero_velocity_update{}; ///< Stores zero velocity update.

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)
	AuxGlobalPosition _aux_global_position {}; ///< Stores aux global position.
#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION
};

#endif // !EKF_EKF_H
