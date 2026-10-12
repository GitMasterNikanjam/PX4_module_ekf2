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

#ifndef EKF_AUX_GLOBAL_POSITION_HPP
#define EKF_AUX_GLOBAL_POSITION_HPP

/**
 * @file aux_global_position.hpp
 * @brief PX4 uORB adapter for auxiliary global-position aiding sources.
 *
 * This optional adapter subscribes to PX4 auxiliary-position topics and
 * forwards measurements into the EKF. It is compiled only for PX4 module
 * builds with CONFIG_EKF2_AUX_GLOBAL_POSITION and MODULE_NAME defined; it is
 * not part of the host-only EKF2 interface.
 */

#include "../../common.h"

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <lib/parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/aux_global_position.h>
#include <aid_sources/aux_global_position/aux_global_position_control.hpp>

class Ekf;

/**
 * @brief Manages multiple PX4 auxiliary global-position sensor instances.
 *
 * Owns PX4 subscriptions and per-source parameter mapping, buffers delayed
 * samples, and requests fusion through the core Ekf. Sensor slots are limited
 * by MAX_AGP_IDS.
 */
class AuxGlobalPosition : public ModuleParams
{
public:
	static constexpr uint8_t MAX_AGP_IDS = 4; ///< Compile-time constant defining MAX AGP IDS.

	/** @brief Construct the PX4 auxiliary global-position subscription manager. */
	AuxGlobalPosition();
	/** @brief Release per-source subscriptions and allocated source objects. */
	~AuxGlobalPosition();

	/** @brief Poll, delay-align, and fuse available auxiliary global-position samples. */
	void update(Ekf &ekf, const estimator::imuSample &imu_delayed);

	/** @brief Refresh module parameters and update each active source configuration. */
	void updateParameters()
	{
		updateParams();
	}

	/** @brief Return maximum filtered test ratio across active auxiliary position sources. */
	float testRatioFiltered() const;
	/** @brief Return whether at least one auxiliary source is currently fusing. */
	bool anySourceFusing() const;
	/** @brief Return a bitmask indicating which source slots are currently fusing. */
	uint8_t sourceFusingBitmask() const;
	/** @brief Read sensor identifier configured for one parameter instance. */
	int32_t getIdParam(int instance);
	/** @brief Set sensor identifier associated with one parameter instance. */
	void setIdParam(int instance, int32_t sensor_id);
	/** @brief Map a sensor identifier to its managed slot, or a negative value if absent. */
	int mapSensorIdToSlot(int32_t sensor_id);
	/** @brief Reconcile subscriptions and source state after parameter changes. */
	void paramsUpdated();

private:
	AgpSource *_sources[MAX_AGP_IDS] {}; ///< Stores sources.
	uORB::Subscription _agp_sub[MAX_AGP_IDS]; ///< Stores agp sub.
	int8_t _instance_slot_map[MAX_AGP_IDS] {-1, -1, -1, -1}; ///< Stores instance slot map.
	uint8_t _n_sources{0}; ///< Stores n sources.

	/** @brief Read an integer auxiliary-position parameter for one source instance. */
	int32_t getAgpParamInt32(const char *param_suffix, int instance) const;
	/** @brief Write an integer auxiliary-position parameter for one source instance. */
	bool setAgpParamInt32(const char *param_suffix, int instance, int32_t value);

	int32_t _id_param_values[MAX_AGP_IDS] {}; ///< Stores id param values.

};

#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION && MODULE_NAME

#endif // !EKF_AUX_GLOBAL_POSITION_HPP
