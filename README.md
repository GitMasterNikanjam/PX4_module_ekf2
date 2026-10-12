# Standalone EKF2 Library

This directory contains a host-usable C++ implementation of the PX4 EKF2 estimator. It retains the estimator's original EKF core and sensor-fusion algorithms, with a small host-facing API for submitting measurements and advancing the filter. The library is intended to be linked into another application; it does not need to run as a PX4 module.

This guide describes the library's architecture, algorithms, build options, data contracts, integration sequence, outputs, and example program. The primary API is in [`EKF2.hpp`](EKF2.hpp), the estimator core is in [`EKF/ekf.h`](EKF/ekf.h), and sensor sample types, units, coordinate frames, and parameters are defined in [`EKF/common.h`](EKF/common.h).

## What the library does

The estimator propagates attitude, velocity, position, and sensor-bias states from IMU increments. It then uses available aiding measurements to correct those estimates and their uncertainty. Depending on build features and run-time configuration, aiding can come from GNSS, a barometer, a magnetometer, airspeed, range finder, optical flow, external vision, auxiliary velocity, drag, gravity, ranging beacons, and synthetic constraints such as zero velocity or sideslip.

The filter includes delayed sensor fusion: non-IMU measurements are buffered and aligned with the estimator's delayed fusion horizon. A separate output predictor propagates an estimate forward to the latest IMU time and smoothly applies corrections from the delayed filter. This gives applications both the filter state at its fusion horizon and a low-latency output.

The library does **not** acquire or calibrate sensors, schedule its own updates, synchronize concurrent callers, publish messages, persist parameters, or select device drivers. The host application supplies correctly timestamped and converted samples, calls the API, configures parameters and vehicle context, and consumes or publishes the results. If sensor correction is needed, perform it in the host before submitting samples.

## Architecture and source layout

| Component | Location | Purpose |
|---|---|---|
| Host facade | [`EKF2.hpp`](EKF2.hpp), [`EKF2.cpp`](EKF2.cpp) | Accepts an `InputFrame`, forwards newly available sensor samples to the core, adds the IMU sample, and advances the estimator. |
| Multi-instance policy | [`EKF2Selector.hpp`](EKF2Selector.hpp), [`EKF2Selector.cpp`](EKF2Selector.cpp) | Selects among host-managed EKFs from host-provided health, fault, and score values. It does not own or update EKF instances. |
| EKF core | [`EKF/ekf.h`](EKF/ekf.h) and the `.cpp` files under `EKF/` | Maintains nominal navigation states and covariance, predicts from IMU, runs fusion control, and exposes estimates and diagnostics. |
| Shared data contract | [`EKF/common.h`](EKF/common.h) | Defines sensor samples, run-time fusion controls, estimator parameters, status flags, and frame enums. |
| Delayed input interface | [`EKF/estimator_interface.h`](EKF/estimator_interface.h) | Buffers sensor data, maintains common estimator state and controls, and provides input/output accessors. |
| Sensor aiding | [`EKF/aid_sources/`](EKF/aid_sources/) | Contains sensor-specific checks, fusion control, state resets, and measurement updates. Most sources are built only when their feature is enabled. |
| Bias estimation | [`EKF/bias_estimator/`](EKF/bias_estimator/) | Scalar random-walk bias estimators for height and horizontal position aiding sources. |
| IMU down-sampling | [`EKF/imu_down_sampler/`](EKF/imu_down_sampler/) | Combines incoming IMU increments when the configured prediction period is slower than the input rate. |
| Output prediction | [`EKF/output_predictor/`](EKF/output_predictor/) | Propagates low-latency output states and tracks delayed EKF corrections. |
| Sample buffers | [`EKF/ringbuffer/`](EKF/ringbuffer/) | Timestamped circular buffers used to align measurements with the fusion horizon. |
| Emergency yaw estimate | [`EKF/yaw_estimator/`](EKF/yaw_estimator/) | Gaussian-sum yaw estimator with multiple yaw hypotheses and velocity-based likelihood weighting. Built with GNSS support. |
| Generated equations | [`EKF/ekf_derivation/generated/`](EKF/ekf_derivation/generated/) | Generated state definitions and covariance/Jacobian equations used by the core. Keep these files consistent with their generator. |

`EKF/aid_sources/aux_global_position/` contains an optional PX4-specific uORB adapter. It is not part of the standalone host API and is not enabled by the standalone CMake feature options.

## Estimation algorithms

### Inertial prediction and error-state correction

The EKF uses a nominal navigation state, including a quaternion attitude, velocity, position, and enabled bias/environment states, together with a covariance over state errors. Each IMU sample supplies integrated delta angle and delta velocity plus their integration intervals. The core propagates the nominal state and covariance using those increments and process-noise parameters.

For an aiding observation, the relevant source computes a predicted observation, innovation, innovation variance, and a normalized consistency test ratio. The estimator applies configured gates and source-specific health rules before updating the state and covariance. Depending on source and state validity, it can also reset selected states directly to an observation. The status accessors expose observations, innovations, variances, test ratios, rejection state, and whether fusion occurred.

### Delayed fusion and low-latency output

Sensor samples have independent timestamps and are buffered. The EKF consumes them at a delayed time horizon to account for sensor latency and configured sensor delays. The output predictor separately integrates IMU data to the current time, stores a short history, and applies corrections corresponding to the delayed EKF state. Its outputs are suitable for low-latency use; `Ekf::state()` describes the delayed filter state.

### Sensor-specific algorithms

- **GNSS:** quality and integrity checks cover fix type, satellite count, dilution of precision, reported accuracy, drift, speed, spoofing, and jamming. Position, velocity, altitude, and optional dual-antenna heading can be fused independently as configured. The optional Gaussian-sum yaw estimator can recover a yaw estimate from inertial prediction and horizontal velocity observations.
- **Barometer and height aiding:** pressure altitude can constrain vertical position. Height-source bias estimators track offsets relative to the selected height reference. Other enabled height sources can include GNSS, range, and external vision.
- **Magnetometer:** the core supports heading or three-axis magnetic-field fusion, consistency checks, field-state estimation, and declination handling. The selected mode and checks depend on parameters and feature configuration.
- **External vision:** position, velocity, and yaw can be fused with supplied frames, reset counters, quality, and variances. Velocity adapters convert supported frames and account for sensor lever arms.
- **Range finder and terrain:** range measurements are checked for validity, freshness, quality, tilt, continuity, fog/blocked returns, and stuck readings. A kinematic check compares range-derived vertical motion with inertial motion. Range and optical flow can support terrain estimation when enabled.
- **Optical flow:** image angular rates are combined with range/height and gyro information to estimate horizontal motion. Sensor rate and distance limits are supplied by the host or parameters.
- **Airspeed, drag, gravity, sideslip, auxiliary velocity, and ranging beacons:** these are optional aiding paths that add motion or environmental constraints. Their input and tuning structures are available only when their compile-time feature is enabled.
- **Zero-motion constraints:** stationary logic can apply zero-angular-rate and zero-velocity updates to reduce inertial drift. The host supplies in-air and at-rest context where needed.

This is an estimator, not a sensor-fusion framework that discovers or configures devices automatically. Sensor availability, calibration, measurement covariance, timing, and vehicle context remain integration responsibilities.

## Build

The repository root CMake project can build the EKF core and host API without a separate PX4-Autopilot checkout. The generated equations and timestamped ring buffer are included in this repository. The build uses the extracted math, matrix, geo, latitude/longitude/altitude, atmosphere, and world magnetic model support libraries in this workspace.

From the repository root:

```sh
cmake -S . -B build-ekf2 \
  -DPX4_EKF2_BUILD_STANDALONE=ON \
  -DPX4_EKF2_BUILD_EXAMPLE=ON
cmake --build build-ekf2 --target ekf2_standalone_example
```

The main CMake target is `PX4::ekf2`. Aliases `PX4::ekf2_core` and `ecl_EKF` refer to the same estimator library. The target requires C++17 and propagates its public include paths, enabled feature definitions, and dependent library targets to consumers.

To consume it from a CMake project that includes this repository as a subdirectory, enable the standalone option before adding the repository and link the target:

```cmake
set(PX4_EKF2_BUILD_STANDALONE ON CACHE BOOL "Build standalone EKF2")
add_subdirectory(path/to/PX4_Libraries)

add_executable(my_navigation_app main.cpp)
target_link_libraries(my_navigation_app PRIVATE PX4::ekf2)
```

The root project builds the other extracted libraries as well, so their source directories and the shared `PX4_mathlib` and `PX4_matrix` dependencies must be present. Override dependency paths with the root options `PX4_MATHLIB_DIR` and `PX4_MATRIX_DIR` when they are stored elsewhere.

## Build-time features

Standalone options are declared in `PX4_module_ekf2/CMakeLists.txt`. Optional sensor structures and API methods are guarded by the matching `CONFIG_EKF2_*` definitions, which CMake publishes to dependents when enabled. The options are enabled by default except auxiliary global position, which is a PX4 uORB adapter and is not a standalone feature.

| CMake option | Enables |
|---|---|
| `PX4_EKF2_ENABLE_AIRSPEED` | Airspeed fusion |
| `PX4_EKF2_ENABLE_AUXVEL` | Auxiliary velocity fusion |
| `PX4_EKF2_ENABLE_BAROMETER` | Barometric height fusion |
| `PX4_EKF2_ENABLE_DRAG_FUSION` | Drag-based motion aiding |
| `PX4_EKF2_ENABLE_EXTERNAL_VISION` | External-vision position, velocity, and yaw paths |
| `PX4_EKF2_ENABLE_GNSS` | GNSS health checks and position/velocity/height fusion |
| `PX4_EKF2_ENABLE_GNSS_YAW` | GNSS yaw estimator/fusion; requires GNSS |
| `PX4_EKF2_ENABLE_GRAVITY_FUSION` | Gravity-vector aiding |
| `PX4_EKF2_ENABLE_MAGNETOMETER` | Magnetometer fusion and magnetic-field states |
| `PX4_EKF2_ENABLE_OPTICAL_FLOW` | Optical-flow fusion |
| `PX4_EKF2_ENABLE_RANGE_FINDER` | Range-finder fusion and consistency checks |
| `PX4_EKF2_ENABLE_RANGING_BEACON` | Ranging-beacon fusion |
| `PX4_EKF2_ENABLE_SIDESLIP` | Synthetic sideslip fusion |
| `PX4_EKF2_ENABLE_TERRAIN` | Terrain estimation |
| `PX4_EKF2_ENABLE_WIND` | Wind-state estimation |

For example, disable unused sensor paths at configure time:

```sh
cmake -S . -B build-minimal \
  -DPX4_EKF2_BUILD_STANDALONE=ON \
  -DPX4_EKF2_ENABLE_OPTICAL_FLOW=OFF \
  -DPX4_EKF2_ENABLE_RANGE_FINDER=OFF \
  -DPX4_EKF2_ENABLE_EXTERNAL_VISION=OFF
```

The producer and consumer must use the same feature configuration because the enabled features determine which sample fields and methods exist in public headers. GNSS yaw is only enabled when both GNSS and GNSS yaw are enabled.

## Basic integration sequence

1. **Build with the required features.** Enable every sensor path the application intends to submit and fuse.
2. **Create one `EKF2` object per filter instance.** Keep each instance owned and updated by the host.
3. **Configure parameters.** Set prediction interval, observation delays, process/measurement noise, innovation gates, sensor offsets, and source modes to match the hardware and application.
4. **Set fusion availability and enable flags.** A source is intended for fusion when both its `FusionSensor::available` and `FusionSensor::enabled` values are true. Set these in `EKF2::fusionControl()` consistently with the connected/configured sources. Parameters also control source modes and conditions.
5. **Initialize with a monotonic microsecond timestamp.** Call `init()` before the first update.
6. **Provide vehicle context.** Set `system_flags` when vehicle state changes, or use the relevant `Ekf`/`EstimatorInterface` setters for advanced integration.
7. **Submit each IMU sample.** Every `update()` frame must contain an IMU increment. Add optional aiding fields only when new samples are available.
8. **Check update and validity status.** Check the return value, attitude alignment, solution status, fault flags, and aiding diagnostics as appropriate.
9. **Consume outputs.** Read the low-latency predictor outputs or the delayed EKF state according to the needs of the application.

### Minimal C++ example

```cpp
#include <EKF2.hpp>

int main()
{
    EKF2 filter;

    // Example only: tune these values and enable flags for the real sensors.
    filter.parameters().ekf2_predict_us = 10'000;
    filter.fusionControl().gps.available = true;
    filter.fusionControl().gps.enabled = true;

    uint64_t time_us = 1'000'000; // Use the application's monotonic clock.
    if (!filter.init(time_us)) {
        return 1;
    }

    while (read_next_imu_sample()) {
        EKF2::InputFrame frame{};
        frame.imu.time_us = time_us;
        frame.imu.delta_ang = imu_delta_angle_body_rad;
        frame.imu.delta_vel = imu_delta_velocity_body_m_s;
        frame.imu.delta_ang_dt = imu_delta_angle_interval_s;
        frame.imu.delta_vel_dt = imu_delta_velocity_interval_s;

        if (gnss_has_new_sample()) {
            frame.gnss = make_gnss_sample();
        }

        if (!filter.update(frame)) {
            // Handle an unavailable or unsuccessful estimator update.
        }

        time_us += imu_period_us;

        const auto &ekf = filter.estimator();
        const matrix::Quatf &attitude = ekf.getQuaternion();
        const matrix::Vector3f velocity_ned = ekf.getVelocity();
        const matrix::Vector3f position_ned = ekf.getPosition();
        (void)attitude;
        (void)velocity_ned;
        (void)position_ned;
    }
}
```

`read_next_imu_sample()`, `make_gnss_sample()`, and the IMU variables above are placeholders for host-specific driver code. See [`examples/standalone_ekf2.cpp`](examples/standalone_ekf2.cpp) for a buildable, intentionally small integration loop. Its synthetic stationary input demonstrates API flow only; it is not a flight-ready sensor configuration or a substitute for tuning.

## Input samples, units, and frames

The sample structures are defined in `EKF/common.h`; the complete per-field descriptions there are authoritative. All timestamps supplied to the estimator must be expressed in the same monotonic microsecond time base. Measurement integration periods and delays are in seconds or milliseconds as specified by each field/parameter.

| Sample | Main contents | Important conventions |
|---|---|---|
| `imuSample` | Integrated delta angle, delta velocity, integration periods, clipping flags | Delta angle in radians and delta velocity in m/s, expressed in body axes. Submit one per update. |
| `gnssSample` | Latitude/longitude, altitude, NED velocity, accuracies, fix/quality, optional yaw | Latitude/longitude in degrees; altitude above mean sea level in metres; velocity in local NED in m/s. |
| `magSample` | Magnetic field vector | Body/sensor axes; Gauss. |
| `baroSample` | Pressure altitude | Metres above sea level. |
| `airspeedSample` | True airspeed and EAS-to-TAS factor | Speed in m/s. |
| `flowSample` | Integrated image angular rate and optional gyro rate | Angular rate in rad/s; timestamp refers to the integration midpoint. |
| `extVisionSample` | Position, velocity, attitude, uncertainty, frames, quality and reset counter | Explicit position/velocity frame enums; positions in m, velocity in m/s, attitude in quaternion form. |
| `auxVelSample` | Horizontal velocity and variance | North/east velocity in local frame, m/s; variance in (m/s)^2. |
| `rangingBeaconSample` | Beacon ID, measured range, variance, and geodetic beacon location | Range in metres; coordinates in degrees and altitude AMSL in metres. |
| `systemFlagUpdate` | Timestamped vehicle context | At-rest, in-air, fixed-wing, ground-effect, constant-position, and transition flags. |

Sensor drivers must convert native sensor messages into these units and frames before submission. Set unsupported optional measurements to empty; an absent optional value means “no new sample this cycle,” not “disable fusion.” Disable a source through fusion controls/parameters when it should no longer be used.

## Configuration and run-time controls

`EKF2::parameters()` returns the estimator's mutable `estimator::parameters`. Parameters cover prediction timing, sensor delays, process and observation noise, innovation gates, sensor modes, mounting offsets, GNSS health limits, and other estimator behavior. Names retain the EKF2 parameter naming convention; descriptions, units, defaults, and feature guards are in `EKF/common.h`.

`EKF2::fusionControl()` returns `estimator::FusionControl`. Each `FusionSensor` contains:

- `available`: the source is present and configured in this application.
- `enabled`: the host currently permits its use.
- `intended()`: true only when both values are true.

The host should keep these flags consistent with actual data availability and its selected operating mode. They do not replace sensor quality checks or source-specific control parameters.

Vehicle context can be sent with `InputFrame::system_flags`, or set through `filter.estimator()` when needed. At-rest and in-air state are particularly important for zero-motion constraints and some GNSS checks. Set context changes with timestamps on the same clock as sensor data.

## Outputs and diagnostics

Use `filter.estimator()` to access the original `Ekf` API. Common outputs include:

- `getQuaternion()` for low-latency attitude.
- `getVelocity()` and `getPosition()` for low-latency local NED velocity and position.
- `getLatLonAlt()` for low-latency global position.
- `state()` for the EKF state at the delayed fusion horizon.
- `covariances()`, `covariances_diagonal()`, and typed `getStateVariance<State::...>()` for uncertainty.
- `attitude_valid()`, local/global position-validity accessors, and `get_ekf_soln_status()` for validity.
- `control_status()`, `fault_status()`, and `information_event_status()` for mode, fault, and reset/event flags.
- `aid_src_*()` accessors for sensor observations, innovations, innovation variances, test ratios, rejection state, and fusion state.

An innovation test ratio is a normalized residual measure used by the corresponding gate. Interpret each aiding status using its source units and gate behavior. A healthy sensor input does not guarantee that a sample was accepted: it may be rejected by health checks or innovation consistency checks.

The output predictor is updated as part of the estimator cycle. Applications should use its low-latency outputs for responsive control and the delayed `state()` / aiding diagnostics when they need the filter's fusion-horizon estimate. Confirm validity before treating any output as navigation-ready.

## Multiple EKF instances

`EKF2Selector` is an independent policy helper; it does not create, schedule, feed, or own filters. The host supplies an `InstanceStatus` array containing `healthy`, `fault`, and `error_score` for each estimator and calls `update()` with the current monotonic time.

Automatic selection prefers the healthy, non-faulted instance with the lowest finite score. A challenger replaces a healthy selection only if its score is less than half the current score and the selection has remained stable for at least one second. An unhealthy or faulted selected instance is replaced immediately. A healthy requested index takes precedence over automatic scoring. If no usable candidate exists, the return value is `INVALID_INSTANCE`.

The host defines the score. A useful score can combine relevant filtered innovation ratios, rejected-fusion counts, estimator validity, and other application-specific quality measures. Keep score scales comparable across instances and avoid non-finite scores for automatic candidates.

## Example program

The buildable example in [`examples/standalone_ekf2.cpp`](examples/standalone_ekf2.cpp) shows how to:

1. Construct and initialize an estimator.
2. Fill timestamped IMU increments and vehicle flags.
3. Call `update()` on a host-controlled loop.
4. Read the core state and submit instance health to `EKF2Selector`.

The example is a structural demonstration. Replace its synthetic IMU values with calibrated and frame-converted measurements from the target application, set realistic sensor noise and delays, and validate estimator outputs with recorded data before operational use.

## Threading and lifecycle

Each `EKF2` instance is mutable and is intended to be accessed serially by its owner. The library does not lock around `init()`, `update()`, parameter changes, sensor setters, or output reads. Do not update the same instance concurrently. If another thread needs estimates, publish a copied snapshot through synchronization owned by the application.

Call `init()` once before submitting measurements. Use monotonically increasing timestamps without mixing clock domains. Submit IMU increments in chronological order and preserve their integration periods. Optional aiding samples should also have valid timestamps and be ordered. Configure buffers and delay-related parameters before operation; significant run-time changes to delays or prediction period may require reinitialization depending on the application.

## Troubleshooting checklist

- **Headers do not expose a sensor field:** enable its `PX4_EKF2_ENABLE_*` CMake option and rebuild both the library and consumer with the same configuration.
- **Update does not produce a usable estimate:** confirm `init()` succeeded, every frame has a valid IMU sample, timestamps advance, and integration intervals are positive and realistic.
- **A source is not fusing:** confirm its feature is compiled, both `available` and `enabled` are true, the matching parameters permit fusion, samples are fresh and correctly framed, and health/innovation checks pass.
- **Position/attitude appears wrong:** check units, body and earth frames, NED/FRD conventions, sensor lever arms, heading conventions, and timestamps/delays.
- **Outputs lag:** distinguish delayed `state()` from the output predictor's current-time estimate; use the intended output path and verify the output predictor is receiving each IMU increment.
- **Standalone CMake configuration fails:** check that the extracted mathlib and matrix dependencies exist at the configured paths and that this repository's support libraries are present.

## License and upstream attribution

The estimator algorithms and generated equations retain PX4 Development Team copyright and licensing notices in their source files. Review the notices in each file before redistributing modified source or binaries.
