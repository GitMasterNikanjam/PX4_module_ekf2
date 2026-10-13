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
 * @file estimator_interface.h
 * @brief Sensor input buffers, runtime controls, and shared estimator state.
 *
 * This base class receives timestamped sensor samples, maintains the delayed
 * sample buffers used by the EKF, and owns common parameters and fusion
 * controls. It is an implementation interface for Ekf; most applications
 * should interact through `EKF2` and access this class indirectly through
 * `Ekf` when advanced controls are needed. Sensor setters are conditional on
 * the matching `CONFIG_EKF2_*` build feature.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

#ifndef EKF_ESTIMATOR_INTERFACE_H
#define EKF_ESTIMATOR_INTERFACE_H

#include <cstdio>
#include <cinttypes>

# define ECL_INFO(X, ...) printf(X "\n", ##__VA_ARGS__)
# define ECL_WARN(X, ...) fprintf(stderr, X "\n", ##__VA_ARGS__)
# define ECL_ERR(X, ...) fprintf(stderr, X "\n", ##__VA_ARGS__)

# if defined(DEBUG_BUILD)
#  define ECL_DEBUG(X, ...) fprintf(stderr, X "\n", ##__VA_ARGS__)
# else
#  define ECL_DEBUG(X, ...)
#endif

#include "common.h"
#include "ringbuffer/TimestampedRingBuffer.hpp"
#include "imu_down_sampler/imu_down_sampler.hpp"
#include "output_predictor/output_predictor.h"

#if defined(CONFIG_EKF2_RANGE_FINDER)
# include "aid_sources/range_finder/range_finder_consistency_check.hpp"
# include "aid_sources/range_finder/sensor_range_finder.hpp"
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_GNSS)
# include "aid_sources/gnss/gnss_checks.hpp"
#endif // CONFIG_EKF2_GNSS

#include <atmosphere.h>
#include <lat_lon_alt.hpp>
#include <src/math.hpp>
#include <mathlib.h>
#include <math/filter/AlphaFilter.hpp>

using namespace estimator;

/**
 * @brief Shared input and control layer for the EKF estimator.
 *
 * IMU increments drive the estimator timeline. Other sensor measurements are
 * timestamped independently and buffered until they can be aligned with the
 * delayed fusion horizon. Set vehicle context (for example, in-air and
 * at-rest state) from the host when it changes. Parameters and fusion control
 * are mutable runtime state and should be configured consistently with the
 * enabled build features.
 */
class EstimatorInterface
{
public:
	/** @brief Add an integrated body-frame IMU delta sample to the input buffer. */
	void setIMUData(const imuSample &imu_sample);

#if defined(CONFIG_EKF2_GNSS)
	/** @brief Add a GNSS sample; timestamp and frames follow gnssSample documentation. */
	void setGpsData(const gnssSample &gnss_sample);

	/** @brief Return the GNSS sample selected at the delayed fusion horizon. */
	const gnssSample &get_gps_sample_delayed() const { return _gps_sample_delayed; }

	/** @brief Get filtered GNSS horizontal position drift rate in m/s. */
	float gps_horizontal_position_drift_rate_m_s() const { return _gnss_checks.horizontal_position_drift_rate_m_s(); }
	/** @brief Get filtered GNSS vertical position drift rate in m/s. */
	float gps_vertical_position_drift_rate_m_s() const { return _gnss_checks.vertical_position_drift_rate_m_s(); }
	/** @brief Get filtered GNSS horizontal speed in m/s. */
	float gps_filtered_horizontal_velocity_m_s() const { return _gnss_checks.filtered_horizontal_velocity_m_s(); }

#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
	/** @brief Add a body-frame magnetometer sample to its delayed buffer. */
	void setMagData(const magSample &mag_sample);
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_BAROMETER)
	/** @brief Add a pressure-altitude sample to its delayed buffer. */
	void setBaroData(const baroSample &baro_sample);
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AIRSPEED)
	/** @brief Add an airspeed measurement to its delayed buffer. */
	void setAirspeedData(const airspeedSample &airspeed_sample);
	/** @brief Mark whether the supplied airspeed source is synthetic. */
	void setSyntheticAirspeed(const bool synthetic_airspeed) { _synthetic_airspeed = synthetic_airspeed; }
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_RANGE_FINDER)
	/** @brief Add a range-finder sample to the delayed measurement buffer. */
	void setRangeData(const estimator::sensor::rangeSample &range_sample);

	/** @brief Set minimum and maximum sensor range in metres. */
	void set_rangefinder_limits(float min_distance, float max_distance)
	{
		_range_sensor.setLimits(min_distance, max_distance);
	}

	/** @brief Return the range sample aligned to the delayed fusion horizon. */
	const estimator::sensor::rangeSample &get_rng_sample_delayed() { return *(_range_sensor.getSampleAddress()); }
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	/** @brief Add optical flow; set gyro_rate to NaN if the sensor has no gyro data. */
	void setOpticalFlowData(const flowSample &flow);

	/** @brief Set flow-rate limit (rad/s) and valid distance interval (metres). */
	void set_optical_flow_limits(float max_flow_rate, float min_distance, float max_distance)
	{
		_flow_max_rate = max_flow_rate;
		_flow_min_distance = min_distance;
		_flow_max_distance = max_distance;
	}
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	/** @brief Add external-vision pose/velocity data to its delayed buffer. */
	void setExtVisionData(const extVisionSample &evdata);
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUXVEL)
	/** @brief Add an auxiliary velocity measurement to its delayed buffer. */
	void setAuxVelData(const auxVelSample &auxvel_sample);
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_RANGING_BEACON)
	/** @brief Add a ranging-beacon observation to its delayed buffer. */
	void setRangingBeaconData(const rangingBeaconSample &ranging_beacon_sample);
#endif // CONFIG_EKF2_RANGING_BEACON

	/** @brief Apply timestamped vehicle-context flags supplied by the host. */
	void setSystemFlagData(const systemFlagUpdate &system_flags);

	/** @brief Get mutable EKF configuration parameters. */
	parameters *getParamHandle() { return &_params; }
	/** @brief Get mutable run-time sensor fusion controls. */
	FusionControl *getFusionControlHandle() { return &_fc; }

	/** @brief Set airborne state and update takeoff/landing timing. */
	void set_in_air_status(bool in_air)
	{
		if (!in_air) {
			if (_control_status.flags.in_air) {
				ECL_DEBUG("no longer in air");
			}

			_time_last_on_ground_us = _time_delayed_us;

		} else {
			if (!_control_status.flags.in_air) {
				ECL_DEBUG("in air");
			}

			_time_last_in_air = _time_delayed_us;
		}

		_control_status.flags.in_air = in_air;
	}

	/** @brief Set whether the host considers the vehicle stationary. */
	void set_vehicle_at_rest(bool at_rest)
	{
		if (!_control_status.flags.vehicle_at_rest && at_rest) {
			ECL_DEBUG("at rest");

		} else if (_control_status.flags.vehicle_at_rest && !at_rest) {
			ECL_DEBUG("no longer at rest");
		}

		_control_status.flags.vehicle_at_rest = at_rest;
	}

	/** @brief Set the constant-position context flag used by estimator controls. */
	void set_constant_pos(bool constant_pos) { _control_status.flags.constant_pos = constant_pos; }

	/** @brief Return true once tilt alignment has completed. */
	bool attitude_valid() const { return _control_status.flags.tilt_align; }

#if defined(CONFIG_EKF2_WIND)
	/** @brief Return whether wind estimation is active or externally initialized. */
	bool get_wind_status() const { return _control_status.flags.wind || _external_wind_init; }
#endif // CONFIG_EKF2_WIND

	/** @brief Set the fixed-wing context flag used by fusion logic. */
	void set_is_fixed_wing(bool is_fixed_wing) { _control_status.flags.fixed_wing = is_fixed_wing; }

	/** @brief Set whether a VTOL transition is in progress. */
	void set_in_transition(bool in_transition) { _control_status.flags.in_transition = in_transition; }

	// set flag if static pressure rise due to ground effect is expected
	// use _params.ekf2_gnd_eff_dz to adjust for expected rise in static pressure
	// flag will clear after GNDEFFECT_TIMEOUT uSec
	/** @brief Set gnd effect. */
	void set_gnd_effect()
	{
		_control_status.flags.gnd_effect = true;
		_time_last_gnd_effect_on = _time_delayed_us;
	}

	/** @brief Set air density in kg/m^3 for drag-force fusion. */
	void set_air_density(float air_density) { _air_density = air_density; }

	/** @brief Return true if the flagged source is the only active horizontal aid. */
	bool isOnlyActiveSourceOfHorizontalAiding(bool aiding_flag) const;
	/** @brief Return true if the flagged source is the only active horizontal-position aid. */
	bool isOnlyActiveSourceOfHorizontalPositionAiding(bool aiding_flag) const;
	/** @brief Return true if the flagged source is the only active horizontal-velocity aid. */
	bool isOnlyActiveSourceOfHorizontalVelocityAiding(bool aiding_flag) const;

	/*
	 * Check if there are any other active source of horizontal aiding
	 * Warning: does not tell if the selected source is
	 * active, use isOnlyActiveSourceOfHorizontalAiding() for this
	 *
	 * The flags considered are opt_flow, gps, ev_vel and ev_pos
	 *
	 * @param aiding_flag a flag in _control_status.flags
	 * @return true if an other source than aiding_flag is active
	 */
	/** @brief Is other source of horizontal aiding than. */
	bool isOtherSourceOfHorizontalAidingThan(bool aiding_flag) const;
	/** @brief Check for another active horizontal-position aid besides the flagged source. */
	bool isOtherSourceOfHorizontalPositionAidingThan(bool aiding_flag) const;
	/** @brief Check for another active horizontal-velocity aid besides the flagged source. */
	bool isOtherSourceOfHorizontalVelocityAidingThan(bool aiding_flag) const;

	// Return true if at least one source of horizontal aiding is active
	// the flags considered are opt_flow, gps, ev_vel and ev_pos
	/** @brief Return whether any supported horizontal aiding is active. */
	bool isHorizontalAidingActive() const;
	/** @brief Return whether any horizontal-position aiding is active. */
	bool isHorizontalPositionAidingActive() const;
	/** @brief Return whether any vertical aiding is active. */
	bool isVerticalAidingActive() const;
	/** @brief Return whether north/east aiding is active. */
	bool isNorthEastAidingActive() const;

	/** @brief Count active horizontal aiding sources. */
	int getNumberOfActiveHorizontalAidingSources() const;
	/** @brief Count active horizontal-position aiding sources. */
	int getNumberOfActiveHorizontalPositionAidingSources() const;
	/** @brief Count active horizontal-velocity aiding sources. */
	int getNumberOfActiveHorizontalVelocityAidingSources() const;

	/** @brief Check for another active vertical-position aid besides the flagged source. */
	bool isOtherSourceOfVerticalPositionAidingThan(bool aiding_flag) const;
	/** @brief Return whether vertical-position aiding is active. */
	bool isVerticalPositionAidingActive() const;
	/** @brief Return whether the flagged source is the only active vertical-position aid. */
	bool isOnlyActiveSourceOfVerticalPositionAiding(const bool aiding_flag) const;
	/** @brief Count active vertical-position aiding sources. */
	int getNumberOfActiveVerticalPositionAidingSources() const;

	/** @brief Return whether vertical-velocity aiding is active. */
	bool isVerticalVelocityAidingActive() const;
	/** @brief Count active vertical-velocity aiding sources. */
	int getNumberOfActiveVerticalVelocityAidingSources() const;

	/** @brief Return the current low-latency body attitude quaternion. */
	const matrix::Quatf &getQuaternion() const { return _output_predictor.getQuaternion(); }
	/** @brief Read and clear accumulated angular velocity from the output predictor. */
	Vector3f getAngularVelocityAndResetAccumulator() { return _output_predictor.getAngularVelocityAndResetAccumulator(); }
	/** @brief Return yaw from bias-corrected gyro integration without aiding. */
	float getUnaidedYaw() const { return _output_predictor.getUnaidedYaw(); }
	/** @brief Return low-latency body-origin velocity in local NED, in m/s. */
	Vector3f getVelocity() const { return _output_predictor.getVelocity(); }

	// get the mean velocity derivative in earth frame since last reset (see `resetVelocityDerivativeAccumulation()`)
	/** @brief Get velocity derivative. */
	Vector3f getVelocityDerivative() const { return _output_predictor.getVelocityDerivative(); }
	/** @brief Clear the accumulated velocity derivative used by output diagnostics. */
	void resetVelocityDerivativeAccumulation() { return _output_predictor.resetVelocityDerivativeAccumulation(); }
	/** @brief Return vertical position derivative in local NED, in m/s. */
	float getVerticalPositionDerivative() const { return _output_predictor.getVerticalPositionDerivative(); }
	/** @brief Return low-latency body-origin position in local NED, in metres. */
	Vector3f getPosition() const;
	/** @brief Return low-latency global latitude, longitude, and altitude. */
	LatLonAlt getLatLonAlt() const { return _output_predictor.getLatLonAlt(); }
	/** @brief Return attitude, velocity, and position output tracking errors. */
	const Vector3f &getOutputTrackingError() const { return _output_predictor.getOutputTrackingError(); }

#if defined(CONFIG_EKF2_MAGNETOMETER)
	/** @brief Get magnetic declination in degrees when available and configured for persistence. */
	bool get_mag_decl_deg(float &val) const
	{
		if (PX4_ISFINITE(_wmm_declination_rad) && (_params.ekf2_decl_type & GeoDeclinationMask::SAVE_GEO_DECL)) {
			val = math::degrees(_wmm_declination_rad);
			return true;

		} else {
			return false;
		}
	}

	/** @brief Get the current geomagnetic inclination in degrees when available. */
	bool get_mag_inc_deg(float &val) const
	{
		if (PX4_ISFINITE(_wmm_inclination_rad)) {
			val = math::degrees(_wmm_inclination_rad);
			return true;

		} else {
			return false;
		}
	}

	/** @brief Return measured/reference field inclination (degrees) and strength (Gauss). */
	void get_mag_checks(float &inc_deg, float &inc_ref_deg, float &strength_gs, float &strength_ref_gs) const
	{
		inc_deg = math::degrees(_mag_inclination);
		inc_ref_deg = math::degrees(_wmm_inclination_rad);
		strength_gs = _mag_strength;
		strength_ref_gs = _wmm_field_strength_gauss;
	}
#endif // CONFIG_EKF2_MAGNETOMETER

	/** @brief Return the complete current EKF control status. */
	const filter_control_status_u &control_status() const { return _control_status; }
	/** @brief Return named flags from the current EKF control status. */
	const decltype(filter_control_status_u::flags) &control_status_flags() const { return _control_status.flags; }

	/** @brief Return control status as saved before the most recent transition. */
	const filter_control_status_u &control_status_prev() const { return _control_status_prev; }
	/** @brief Return named flags from the previous EKF control status. */
	const decltype(filter_control_status_u::flags) &control_status_prev_flags() const { return _control_status_prev.flags; }

	/** @brief Return the complete current EKF fault status. */
	const fault_status_u &fault_status() const { return _fault_status; }
	/** @brief Return named flags from the current EKF fault status. */
	const decltype(fault_status_u::flags) &fault_status_flags() const { return _fault_status.flags; }

	/** @brief Return the complete information-event status. */
	const information_event_status_u &information_event_status() const { return _information_events; }
	/** @brief Return named information-event flags. */
	const decltype(information_event_status_u::flags) &information_event_flags() const { return _information_events.flags; }
	/** @brief Clear all pending information-event flags. */
	void clear_information_events() { _information_events.value = 0; }

	/** @brief Return average EKF update period in seconds. */
	float get_dt_ekf_avg() const { return _dt_ekf_avg; }

	/** @brief Return the oldest IMU sample at the delayed fusion horizon. */
	const imuSample &get_imu_sample_delayed() const { return _imu_buffer.get_oldest(); }
	/** @brief Return delayed fusion-horizon time in microseconds. */
	const uint64_t &time_delayed_us() const { return _time_delayed_us; }

	/** @brief Return whether the global latitude/longitude origin has been initialized. */
	bool global_origin_valid() const { return _local_origin_lat_lon.isInitialized(); }
	/** @brief Return the map projection defining the local origin. */
	const MapProjection &global_origin() const { return _local_origin_lat_lon; }
	/** @brief Return origin altitude in metres, or zero when unavailable. */
	float getEkfGlobalOriginAltitude() const { return PX4_ISFINITE(_local_origin_alt) ? _local_origin_alt : 0.f; }

	/** @brief Access the low-latency output predictor. */
	OutputPredictor &output_predictor() { return _output_predictor; };

protected:

	/** @brief Construct the shared estimator input/control layer. */
	EstimatorInterface() = default;
	/** @brief Release dynamically allocated sample buffers. */
	virtual ~EstimatorInterface();

	virtual bool init(uint64_t timestamp) = 0;

	parameters _params{};		// filter parameters
	FusionControl _fc{}; ///< Stores fc.

	/*
	 OBS_BUFFER_LENGTH defines how many observations (non-IMU measurements) we can buffer
	 which sets the maximum frequency at which we can process non-IMU measurements. Measurements that
	 arrive too soon after the previous measurement will not be processed.
	 max freq (Hz) = (OBS_BUFFER_LENGTH - 1) / (IMU_BUFFER_LENGTH * FILTER_UPDATE_PERIOD_S)
	 This can be adjusted to match the max sensor data rate plus some margin for jitter.
	*/
	uint8_t _obs_buffer_length{0}; ///< Buffer used to retain obs buffer length.

	/*
	IMU_BUFFER_LENGTH defines how many IMU samples we buffer which sets the time delay from current time to the
	EKF fusion time horizon and therefore the maximum sensor time offset relative to the IMU that we can compensate for.
	max sensor time offet (msec) =  IMU_BUFFER_LENGTH * FILTER_UPDATE_PERIOD_MS
	This can be adjusted to a value that is FILTER_UPDATE_PERIOD_MS longer than the maximum observation time delay.
	*/
	uint8_t _imu_buffer_length{0}; ///< Buffer used to retain imu buffer length.

	float _dt_ekf_avg{0.010f}; ///< average update rate of the ekf in s

	uint64_t _time_delayed_us{0}; // captures the imu sample on the delayed time horizon
	uint64_t _time_latest_us{0}; // imu sample capturing the newest imu data

	OutputPredictor _output_predictor{}; ///< Stores output predictor.

#if defined(CONFIG_EKF2_AIRSPEED)
	airspeedSample _airspeed_sample_delayed {}; ///< Stores airspeed sample delayed.
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	extVisionSample _ev_sample_prev {}; ///< Stores ev sample prev.
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_RANGE_FINDER)
	TimestampedRingBuffer<sensor::rangeSample> *_range_buffer {nullptr}; ///< Buffer used to retain range buffer.
	uint64_t _time_last_range_buffer_push{0}; ///< Timestamp or duration value in microseconds: time last range buffer push.

	sensor::SensorRangeFinder _range_sensor{}; ///< Stores range sensor.
	RangeFinderConsistencyCheck _rng_consistency_check; ///< Stores rng consistency check.
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	TimestampedRingBuffer<flowSample> 	*_flow_buffer {nullptr}; ///< Buffer used to retain flow buffer.

	flowSample _flow_sample_delayed{}; ///< Stores flow sample delayed.

	// Sensor limitations
	float _flow_max_rate{1.0f}; ///< maximum angular flow rate that the optical flow sensor can measure (rad/s)
	float _flow_min_distance{0.0f};	///< minimum distance that the optical flow sensor can operate at (m)
	float _flow_max_distance{10.f};	///< maximum distance that the optical flow sensor can operate at (m)
#endif // CONFIG_EKF2_OPTICAL_FLOW

	float _air_density{atmosphere::kAirDensitySeaLevelStandardAtmos};		// air density (kg/m**3)

	bool _imu_updated{false};      // true if the ekf should update (completed downsampling process)
	bool _initialised{false};      // true if the ekf interface instance (data buffering) is initialized

	// Variables used to publish the WGS-84 location of the EKF local NED origin
	MapProjection _local_origin_lat_lon{}; ///< Stores local origin lat lon.
	float _local_origin_alt{NAN}; ///< Stores local origin alt.

#if defined(CONFIG_EKF2_GNSS)
	TimestampedRingBuffer<gnssSample> *_gps_buffer {nullptr}; ///< Buffer used to retain gps buffer.
	uint64_t _time_last_gps_buffer_push{0}; ///< Timestamp or duration value in microseconds: time last gps buffer push.

	gnssSample _gps_sample_delayed{}; ///< Stores gps sample delayed.

	uint32_t _min_gps_health_time_us{10000000}; ///< GPS is marked as healthy only after this amount of time
	GnssChecks _gnss_checks{_params.ekf2_gps_check,
			   _params.ekf2_req_nsats,
			   _params.ekf2_req_pdop,
			   _params.ekf2_req_eph,
			   _params.ekf2_req_epv,
			   _params.ekf2_req_sacc,
			   _params.ekf2_req_hdrift,
			   _params.ekf2_req_vdrift,
			   _params.ekf2_req_fix,
			   _params.ekf2_vel_lim,
			   _min_gps_health_time_us,
			   _control_status};

# if defined(CONFIG_EKF2_GNSS_YAW)
	// innovation consistency check monitoring ratios
	uint64_t _time_last_gnss_yaw_buffer_push{0}; ///< Timestamp or duration value in microseconds: time last gnss yaw buffer push.
# endif // CONFIG_EKF2_GNSS_YAW
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_DRAG_FUSION)
	TimestampedRingBuffer<dragSample> *_drag_buffer {nullptr}; ///< Buffer used to retain drag buffer.
	dragSample _drag_down_sampled{};	// down sampled drag specific force data (filter prediction rate -> observation rate)
#endif // CONFIG_EKF2_DRAG_FUSION

	bool _horizontal_deadreckon_time_exceeded{true}; ///< Timestamp or duration value in microseconds: horizontal deadreckon time exceeded.
	bool _vertical_position_deadreckon_time_exceeded{true}; ///< Timestamp or duration value in microseconds: vertical position deadreckon time exceeded.
	bool _vertical_velocity_deadreckon_time_exceeded{true}; ///< Timestamp or duration value in microseconds: vertical velocity deadreckon time exceeded.

	uint64_t _time_last_on_ground_us{0};	///< last time we were on the ground (uSec)
	uint64_t _time_last_in_air{0};		///< last time we were in air (uSec)

	// data buffer instances
	static constexpr uint8_t kBufferLengthDefault = 12; ///< Compile-time constant defining kBufferLengthDefault.
	TimestampedRingBuffer<imuSample> _imu_buffer{kBufferLengthDefault}; ///< Buffer used to retain imu buffer.

#if defined(CONFIG_EKF2_MAGNETOMETER)
	TimestampedRingBuffer<magSample> *_mag_buffer {nullptr}; ///< Buffer used to retain mag buffer.
	uint64_t _time_last_mag_buffer_push{0}; ///< Timestamp or duration value in microseconds: time last mag buffer push.
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_AIRSPEED)
	TimestampedRingBuffer<airspeedSample> *_airspeed_buffer {nullptr}; ///< Buffer used to retain airspeed buffer.
	bool _synthetic_airspeed{false}; ///< State flag indicating synthetic airspeed.
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	TimestampedRingBuffer<extVisionSample> *_ext_vision_buffer {nullptr}; ///< Buffer used to retain ext vision buffer.
	uint64_t _time_last_ext_vision_buffer_push{0}; ///< Timestamp or duration value in microseconds: time last ext vision buffer push.
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_AUXVEL)
	TimestampedRingBuffer<auxVelSample> *_auxvel_buffer {nullptr}; ///< Buffer used to retain auxvel buffer.
#endif // CONFIG_EKF2_AUXVEL
	TimestampedRingBuffer<systemFlagUpdate> *_system_flag_buffer {nullptr}; ///< Buffer used to retain system flag buffer.

#if defined(CONFIG_EKF2_RANGING_BEACON)
	TimestampedRingBuffer<rangingBeaconSample> *_ranging_beacon_buffer {nullptr}; ///< Buffer used to retain ranging beacon buffer.
	uint64_t _time_last_ranging_beacon_buffer_push{0}; ///< Timestamp or duration value in microseconds: time last ranging beacon buffer push.
#endif // CONFIG_EKF2_RANGING_BEACON

#if defined(CONFIG_EKF2_BAROMETER)
	TimestampedRingBuffer<baroSample> *_baro_buffer {nullptr}; ///< Buffer used to retain baro buffer.
	uint64_t _time_last_baro_buffer_push{0}; ///< Timestamp or duration value in microseconds: time last baro buffer push.
#endif // CONFIG_EKF2_BAROMETER

	uint64_t _time_last_gnd_effect_on{0}; ///< Timestamp or duration value in microseconds: time last gnd effect on.

	fault_status_u _fault_status{}; ///< Stores fault status.

	// allocate data buffers and initialize interface variables
	/** @brief Initialise interface. */
	bool initialise_interface(uint64_t timestamp);

#if defined(CONFIG_EKF2_MAGNETOMETER)
	uint64_t _wmm_mag_time_last_checked {0}; // time WMM update last checked by mag control

	float _wmm_declination_rad{NAN};        // magnetic declination returned by the geo library using the last valid GPS position (rad)
	float _wmm_inclination_rad{NAN};        // magnetic inclination returned by the geo library using the last valid GPS position (rad)
	float _wmm_field_strength_gauss{NAN};   // magnetic strength returned by the geo library using the last valid GPS position (Gauss)

	Vector3f _wmm_earth_field_gauss{};      // expected magnetic field vector from the last valid GPS position (Gauss)

	float _mag_inclination{NAN}; ///< Stores mag inclination.
	float _mag_strength{NAN}; ///< Stores mag strength.
#endif // CONFIG_EKF2_MAGNETOMETER

	bool _external_wind_init{false}; ///< State flag indicating external wind init.

	// this is the current status of the filter control modes
	filter_control_status_u _control_status{}; ///< Stores control status.

	// this is the previous status of the filter control modes - used to detect mode transitions
	filter_control_status_u _control_status_prev{}; ///< Stores control status prev.

	// these are used to record single frame events for external monitoring and should NOT be used for
	// state logic becasue they will be cleared externally after being read.
	information_event_status_u _information_events{}; ///< Stores information events.

	unsigned _min_obs_interval_us{0}; // minimum time interval between observations that will guarantee data is not lost (usec)

#if defined(CONFIG_EKF2_DRAG_FUSION)
	/** @brief Set drag data. */
	void setDragData(const imuSample &imu);

	// Used by the multi-rotor specific drag force fusion
	uint8_t _drag_sample_count{0};	// number of drag specific force samples assumulated at the filter prediction rate
	float _drag_sample_time_dt{0.0f};	// time integral across all samples used to form _drag_down_sampled (sec)
#endif // CONFIG_EKF2_DRAG_FUSION

	/** @brief Print buffer allocation failed. */
	void printBufferAllocationFailed(const char *buffer_name);

	ImuDownSampler _imu_down_sampler{_params.ekf2_predict_us}; ///< Stores imu down sampler.
};
#endif // !EKF_ESTIMATOR_INTERFACE_H
