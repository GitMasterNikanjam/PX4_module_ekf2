#include <EKF2.hpp>
#include <EKF2Selector.hpp>

#include <cstdint>

int main()
{
	EKF2 estimator;
	constexpr uint64_t start_time_us = 1'000'000;

	if (!estimator.init(start_time_us)) {
		return 1;
	}

	EKF2::InputFrame frame{};
	frame.system_flags = estimator::systemFlagUpdate{};
	frame.system_flags->time_us = start_time_us;
	frame.system_flags->at_rest = true;
	frame.system_flags->in_air = false;

	for (uint64_t sample_index = 1; sample_index <= 100; ++sample_index) {
		frame.imu.time_us = start_time_us + sample_index * 10'000;
		frame.imu.delta_ang_dt = 0.01f;
		frame.imu.delta_vel_dt = 0.01f;
		frame.imu.delta_vel(2) = -9.80665f * frame.imu.delta_vel_dt;

		if (!estimator.update(frame)) {
			return 2;
		}
	}

	const auto &state = estimator.estimator().state();
	(void)state;

	EKF2Selector selector;
	const EKF2Selector::InstanceStatus instance_status[] {{true, false, 0.1f}};
	(void)selector.update(instance_status, 1, frame.imu.time_us);
	return 0;
}
