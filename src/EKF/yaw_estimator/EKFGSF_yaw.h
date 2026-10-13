/****************************************************************************
 *
 *   Copyright (c) 2020-2024 PX4 Development Team. All rights reserved.
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

#ifndef EKF_EKFGSF_YAW_H
#define EKF_EKFGSF_YAW_H

#include <mathlib.h>
#include <src/math.hpp>

static constexpr uint8_t N_MODELS_EKFGSF = 5; ///< Compile-time constant defining N MODELS EKFGSF.

/**
 * @brief Gaussian-sum yaw estimator driven by inertial and horizontal velocity data.
 *
 * A bank of yaw hypotheses predicts velocity and weights each hypothesis by
 * its velocity innovation likelihood. The composite yaw estimate can recover
 * heading when the main EKF yaw estimate is uncertain. Inputs use radians,
 * seconds, m/s, and NED horizontal velocity as indicated by method parameters.
 */
class EKFGSF_yaw
{
public:
	/** @brief Construct yaw hypothesis bank in an uninitialized state. */
	EKFGSF_yaw();

	// Update Filter States - this should be called whenever new IMU data is available
	/** @brief Propagate all yaw hypotheses using IMU delta angle and delta velocity. */
	void predict(const matrix::Vector3f &delta_ang, const float delta_ang_dt,
		     const matrix::Vector3f &delta_vel, const float delta_vel_dt,
		     bool in_air = false); ///< State flag indicating in air.

	// vel_NE: NE velocity measurement (m/s)
	// vel_accuracy: 1-sigma accuracy of velocity measurement (m/s)
	/** @brief Fuse local-NED horizontal velocity to reweight and correct yaw hypotheses. */
	void fuseVelocity(const matrix::Vector2f &vel_NE, float vel_accuracy, bool in_air);

	/** @brief Set true airspeed in m/s for fixed-wing acceleration compensation. */
	void setTrueAirspeed(float true_airspeed) { _true_airspeed = true_airspeed; }

	/** @brief Update gyro-bias estimate used by the hypothesis AHRS instances. */
	void setGyroBias(const matrix::Vector3f &imu_gyro_bias, const bool force = false)
	{
		// Initialise to gyro bias estimate from main filter because there could be a large
		// uncorrected rate gyro bias error about the gravity vector
		if (!_ahrs_ekf_gsf_tilt_aligned || !_ekf_gsf_vel_fuse_started || force) {
			// init gyro bias for each model
			for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
				_ahrs_ekf_gsf[model_index].gyro_bias = imu_gyro_bias;
			}
		}
	}

	// get solution data for logging
	/** @brief Copy composite estimate and per-model yaw/innovation/weight diagnostics. */
	bool getLogData(float *yaw_composite,
			float *yaw_composite_variance,
			float yaw[N_MODELS_EKFGSF],
			float innov_VN[N_MODELS_EKFGSF],
			float innov_VE[N_MODELS_EKFGSF],
			float weight[N_MODELS_EKFGSF]) const;

	/** @brief Return whether velocity fusion has initialized and activated the model bank. */
	bool isActive() const { return _ekf_gsf_vel_fuse_started; }

	/** @brief Return composite yaw estimate in radians. */
	float getYaw() const { return _gsf_yaw; }
	/** @brief Return composite yaw variance in rad^2. */
	float getYawVar() const { return _gsf_yaw_variance; }

	/** @brief Reset all yaw hypotheses, weights, and alignment state. */
	void reset();

private:

	// Parameters - these could be made tuneable
	const float _gyro_noise{1.0e-1f}; 	// yaw rate noise used for covariance prediction (rad/sec)
	const float _accel_noise{2.0f};		// horizontal accel noise used for covariance prediction (m/sec**2)
	const float _tilt_gain{0.2f};		// gain from tilt error to gyro correction for complementary filter (1/sec)
	const float _gyro_bias_gain{0.04f};	// gain applied to integral of gyro correction for complementary filter (1/sec)

	// Declarations used by the bank of N_MODELS_EKFGSF AHRS complementary filters
	float _true_airspeed{NAN};	// true airspeed used for centripetal accel compensation (m/s)

	struct {
		matrix::Quatf q{};            // attitude: rotates a vector from body to earth frame
		matrix::Vector3f gyro_bias{}; // gyro bias learned and used by the quaternion calculation
	} _ahrs_ekf_gsf[N_MODELS_EKFGSF] {};

	bool _ahrs_ekf_gsf_tilt_aligned{false};  // true the initial tilt alignment has been calculated
	matrix::Vector3f _ahrs_accel{0.f, 0.f, 0.f};     // low pass filtered body frame specific force vector used by AHRS calculation (m/s/s)

	// calculate the gain from gravity vector misalingment to tilt correction to be used by all AHRS filters
	/** @brief Compute accelerometer correction gain for the auxiliary AHRS. */
	float ahrsCalcAccelGain() const;

	// update specified AHRS rotation matrix using IMU and optionally true airspeed data
	/** @brief Propagate one auxiliary AHRS attitude hypothesis from gyro increments. */
	void ahrsPredict(const uint8_t model_index, const matrix::Vector3f &delta_ang, const float delta_ang_dt);

	// align all AHRS roll and pitch orientations using IMU delta velocity vector
	/** @brief Align auxiliary AHRS tilt from the gravity direction in delta velocity. */
	void ahrsAlignTilt(const matrix::Vector3f &delta_vel);

	// align all AHRS yaw orientations to initial values
	/** @brief Align auxiliary AHRS yaw to its current model hypothesis. */
	void ahrsAlignYaw();

	// Declarations used by a bank of N_MODELS_EKFGSF EKFs

	struct {
		matrix::Vector3f X{};                       // Vel North (m/s),  Vel East (m/s), yaw (rad)s
		matrix::SquareMatrix<float, 3> P{};         // covariance matrix
		float nis{};                                // normalized innovation squared
		float S_det_inverse{};                      // inverse of the innovation covariance matrix determinant
		matrix::Vector2f innov{};                   // Velocity N,E innovation (m/s)
	} _ekf_gsf[N_MODELS_EKFGSF] {};

	bool _ekf_gsf_vel_fuse_started{}; // true when the EKF's have started fusing velocity data and the prediction and update processing is active

	// initialise states and covariance data for the GSF and EKF filters
	/** @brief Initialise ekfgsf. */
	void initialiseEKFGSF(const matrix::Vector2f &vel_NE, const float vel_accuracy);

	// predict state and covariance for the specified EKF using inertial data
	/** @brief Predict ekf. */
	void predictEKF(const uint8_t model_index, const matrix::Vector3f &delta_ang, const float delta_ang_dt,
			const matrix::Vector3f &delta_vel, const float delta_vel_dt, bool in_air = false);

	// update state and covariance for the specified EKF using a NE velocity measurement
	// return false if update failed
	/** @brief Update ekf. */
	bool updateEKF(const uint8_t model_index, const matrix::Vector2f &vel_NE, const float vel_accuracy);

	/** @brief Return scalar square used by yaw-estimator calculations. */
	/** @brief Return square of a scalar value for internal estimator calculations. */
	inline float sq(float x) const { return x * x; };

	// Declarations used by the Gaussian Sum Filter (GSF) that combines the individual EKF yaw estimates

	matrix::Vector<float, N_MODELS_EKFGSF> _model_weights{}; ///< Stores model weights.
	float _gsf_yaw{}; 		// yaw estimate (rad)
	float _gsf_yaw_variance{}; 	// variance of yaw estimate (rad^2)

	// return the probability of the state estimate for the specified EKF assuming a gaussian error distribution
	/** @brief Evaluate the velocity-innovation likelihood for one Gaussian model. */
	float gaussianDensity(const uint8_t model_index) const;
};
#endif // !EKF_EKFGSF_YAW_H
