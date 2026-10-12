/**
 * @file aid_source_status.h
 * @brief Innovation and fusion diagnostics for an aiding source.
 *
 * These compact records expose the observation, innovation, variance, and
 * consistency-test values maintained by the EKF. Scalar and fixed-size
 * vector specializations use the same field meanings.
 */

#ifndef EKF_AID_SOURCE_STATUS_H
#define EKF_AID_SOURCE_STATUS_H

#include <cstdint>

namespace ekf2
{

/**
 * @brief Diagnostics for an N-dimensional aiding measurement.
 * @tparam N Number of measurement axes.
 *
 * Vector fields are axis ordered according to the measurement source. All
 * values use the source's native units; variances use squared units. Times
 * are monotonic microseconds. `test_ratio` is the normalized innovation
 * consistency ratio used by the EKF gate.
 */
template<unsigned N>
struct AidSourceStatus
{
	uint64_t timestamp{}; ///< Time this status was generated (microseconds).
	uint64_t timestamp_sample{}; ///< Timestamp of the aiding observation (microseconds).
	uint64_t time_last_fuse{}; ///< Timestamp of the most recent successful fusion (microseconds).
	uint32_t device_id{}; ///< Source sensor/device identifier, when provided by the integration.
	float observation[N] {}; ///< Latest measurement in source-native units.
	float observation_variance[N] {}; ///< Measurement variances in squared source-native units.
	float innovation[N] {}; ///< Observation minus predicted measurement.
	float innovation_filtered[N] {}; ///< Low-pass filtered innovation.
	float innovation_variance[N] {}; ///< Predicted innovation variances.
	float test_ratio[N] {}; ///< Per-axis normalized innovation test ratio.
	float test_ratio_filtered[N] {}; ///< Low-pass filtered normalized test ratio.
	bool innovation_rejected{false}; ///< True when the innovation consistency gate rejected the sample.
	bool fused{false}; ///< True when this observation was fused during the latest update.
};

/** @brief Scalar specialization of AidSourceStatus with scalar-valued fields. */
template<>
struct AidSourceStatus<1>
{
	uint64_t timestamp{}; ///< Time this status was generated (microseconds).
	uint64_t timestamp_sample{}; ///< Timestamp of the aiding observation (microseconds).
	uint64_t time_last_fuse{}; ///< Timestamp of the most recent successful fusion (microseconds).
	uint32_t device_id{}; ///< Source sensor/device identifier, when provided by the integration.
	float observation{}; ///< Latest measurement in source-native units.
	float observation_variance{}; ///< Measurement variance in squared source-native units.
	float innovation{}; ///< Observation minus predicted measurement.
	float innovation_filtered{}; ///< Low-pass filtered innovation.
	float innovation_variance{}; ///< Predicted innovation variance.
	float test_ratio{}; ///< Normalized innovation consistency test ratio.
	float test_ratio_filtered{}; ///< Low-pass filtered normalized test ratio.
	bool innovation_rejected{false}; ///< True when the innovation consistency gate rejected the sample.
	bool fused{false}; ///< True when this observation was fused during the latest update.
};

/** @brief One-dimensional aiding-source diagnostic record. */
using AidSourceStatus1D = AidSourceStatus<1>;
/** @brief Two-dimensional aiding-source diagnostic record. */
using AidSourceStatus2D = AidSourceStatus<2>;
/** @brief Three-dimensional aiding-source diagnostic record. */
using AidSourceStatus3D = AidSourceStatus<3>;

} // namespace ekf2

#endif // EKF_AID_SOURCE_STATUS_H
