#include "EKF2.hpp"

bool EKF2::init(uint64_t timestamp_us)
{
	return _ekf.init(timestamp_us);
}

bool EKF2::update(const InputFrame &frame)
{
#if defined(CONFIG_EKF2_GNSS)
	if (frame.gnss) { _ekf.setGpsData(*frame.gnss); }
#endif
#if defined(CONFIG_EKF2_MAGNETOMETER)
	if (frame.magnetometer) { _ekf.setMagData(*frame.magnetometer); }
#endif
#if defined(CONFIG_EKF2_BAROMETER)
	if (frame.barometer) { _ekf.setBaroData(*frame.barometer); }
#endif
#if defined(CONFIG_EKF2_AIRSPEED)
	if (frame.airspeed) { _ekf.setAirspeedData(*frame.airspeed); }
#endif
#if defined(CONFIG_EKF2_RANGE_FINDER)
	if (frame.range) { _ekf.setRangeData(*frame.range); }
#endif
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	if (frame.optical_flow) { _ekf.setOpticalFlowData(*frame.optical_flow); }
#endif
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	if (frame.external_vision) { _ekf.setExtVisionData(*frame.external_vision); }
#endif
#if defined(CONFIG_EKF2_AUXVEL)
	if (frame.auxiliary_velocity) { _ekf.setAuxVelData(*frame.auxiliary_velocity); }
#endif
#if defined(CONFIG_EKF2_RANGING_BEACON)
	if (frame.ranging_beacon) { _ekf.setRangingBeaconData(*frame.ranging_beacon); }
#endif
	if (frame.system_flags) { _ekf.setSystemFlagData(*frame.system_flags); }

	_ekf.setIMUData(frame.imu);
	return _ekf.update();
}
