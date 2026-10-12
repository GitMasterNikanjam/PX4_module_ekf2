/**
 * @file EKF2.hpp
 * @brief Host-side input facade for the EKF2 estimator.
 *
 * The facade wraps the original EKF core and accepts timestamped samples
 * directly from a host application. It does not perform sensor acquisition,
 * unit conversion, scheduling, locking, logging, or output publication.
 * Sample units, timestamps, and coordinate frames are defined by
 * `EKF/common.h`. Sensor fields are available only when their corresponding
 * `CONFIG_EKF2_*` feature is enabled at build time.
 */
#pragma once

#include "EKF/ekf.h"

#include <cstdint>
#include <optional>

/**
 * @brief Minimal host integration API over the EKF implementation.
 *
 * Construct one instance per estimator. Call init() once before supplying
 * data, then populate an InputFrame and call update() whenever a new IMU
 * sample is available. Optional aiding samples are submitted alongside that
 * IMU sample; the core buffers and delay-aligns them according to its
 * configured sensor delays. The caller remains responsible for setting
 * parameters and vehicle flags, checking estimator validity, and consuming
 * estimates through estimator().
 *
 * @note Input timestamps must use a consistent monotonic microsecond clock.
 */
class EKF2
{
public:
	/**
	 * @brief Measurements supplied to one estimator update.
	 *
	 * `imu` is required for every call to update(). Each optional sensor
	 * measurement is submitted only when a new sample is available. A missing
	 * optional value means no new measurement is added; it does not disable that
	 * sensor's fusion. `system_flags` is also optional and updates vehicle
	 * context when provided.
	 */
	struct InputFrame {
		estimator::imuSample imu{}; ///< Required integrated IMU increment for this update.
#if defined(CONFIG_EKF2_GNSS)
		std::optional<estimator::gnssSample> gnss; ///< New GNSS position, velocity, and optional heading measurement.
#endif
#if defined(CONFIG_EKF2_MAGNETOMETER)
		std::optional<estimator::magSample> magnetometer; ///< New body-frame magnetic-field measurement.
#endif
#if defined(CONFIG_EKF2_BAROMETER)
		std::optional<estimator::baroSample> barometer; ///< New pressure-altitude measurement.
#endif
#if defined(CONFIG_EKF2_AIRSPEED)
		std::optional<estimator::airspeedSample> airspeed; ///< New true-airspeed measurement.
#endif
#if defined(CONFIG_EKF2_RANGE_FINDER)
		std::optional<estimator::sensor::rangeSample> range; ///< New range-finder measurement.
#endif
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
		std::optional<estimator::flowSample> optical_flow; ///< New optical-flow integration.
#endif
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		std::optional<estimator::extVisionSample> external_vision; ///< New external-vision measurement.
#endif
#if defined(CONFIG_EKF2_AUXVEL)
		std::optional<estimator::auxVelSample> auxiliary_velocity; ///< New auxiliary velocity measurement.
#endif
#if defined(CONFIG_EKF2_RANGING_BEACON)
		std::optional<estimator::rangingBeaconSample> ranging_beacon; ///< New ranging-beacon observation.
#endif
		std::optional<estimator::systemFlagUpdate> system_flags; ///< Updated vehicle state/context flags.
	};

	/** @brief Initialize EKF state and timing.
	 * @param timestamp_us Monotonic startup time in microseconds.
	 * @return True when initialization succeeds.
	 */
	bool init(uint64_t timestamp_us);
	/** @brief Submit a required IMU sample and any newly available aiding data.
	 * @param frame Measurements for this cycle.
	 * @return The core update result; false indicates that no valid update was
	 *         produced (for example, before initialization or with invalid IMU data).
	 */
	bool update(const InputFrame &frame);

	/** @brief Access the underlying EKF core for outputs and advanced controls. */
	Ekf &estimator() { return _ekf; }
	/** @copydoc estimator() */
	const Ekf &estimator() const { return _ekf; }
	/** @brief Access mutable filter parameters; configure before or during operation. */
	estimator::parameters &parameters() { return *_ekf.getParamHandle(); }
	/** @brief Access run-time sensor fusion enable/availability controls. */
	estimator::FusionControl &fusionControl() { return *_ekf.getFusionControlHandle(); }

private:
	Ekf _ekf; ///< Stores ekf.
};
