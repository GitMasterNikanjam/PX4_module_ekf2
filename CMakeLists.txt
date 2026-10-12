# Optional standalone build of the EKF core. EKF2.cpp and EKF2Selector.cpp
# remain PX4 runtime adapters and are intentionally not part of this target.

if(NOT PX4_EKF2_GENERATED_INCLUDE_DIR)
	set(PX4_EKF2_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/EKF")
endif()

if(NOT EXISTS "${PX4_EKF2_GENERATED_INCLUDE_DIR}/ekf_derivation/generated/state.h")
	message(FATAL_ERROR
		"PX4_EKF2_GENERATED_INCLUDE_DIR does not contain "
		"ekf_derivation/generated/state.h: ${PX4_EKF2_GENERATED_INCLUDE_DIR}")
endif()

add_custom_target(prebuild_targets)
add_library(geo ALIAS px4_geo)
add_library(lat_lon_alt ALIAS px4_lat_lon_alt)

option(PX4_EKF2_ENABLE_AIRSPEED "Enable airspeed fusion in the EKF core" ON)
option(PX4_EKF2_ENABLE_AUXVEL "Enable auxiliary velocity fusion" ON)
option(PX4_EKF2_ENABLE_BAROMETER "Enable barometer fusion" ON)
option(PX4_EKF2_ENABLE_DRAG_FUSION "Enable drag fusion" ON)
option(PX4_EKF2_ENABLE_EXTERNAL_VISION "Enable external vision fusion" ON)
option(PX4_EKF2_ENABLE_GNSS "Enable GNSS fusion" ON)
option(PX4_EKF2_ENABLE_GNSS_YAW "Enable GNSS yaw estimator" ON)
option(PX4_EKF2_ENABLE_GRAVITY_FUSION "Enable gravity fusion" ON)
option(PX4_EKF2_ENABLE_MAGNETOMETER "Enable magnetometer fusion" ON)
option(PX4_EKF2_ENABLE_OPTICAL_FLOW "Enable optical flow fusion" ON)
option(PX4_EKF2_ENABLE_RANGE_FINDER "Enable range finder fusion" ON)
option(PX4_EKF2_ENABLE_RANGING_BEACON "Enable ranging beacon fusion" ON)
option(PX4_EKF2_ENABLE_SIDESLIP "Enable sideslip fusion" ON)
option(PX4_EKF2_ENABLE_TERRAIN "Enable terrain estimation" ON)
option(PX4_EKF2_ENABLE_WIND "Enable wind estimation" ON)

set(_ekf2_features
	AIRSPEED AUXVEL BAROMETER DRAG_FUSION EXTERNAL_VISION GNSS GRAVITY_FUSION
	MAGNETOMETER OPTICAL_FLOW RANGE_FINDER RANGING_BEACON SIDESLIP TERRAIN WIND)
set(_ekf2_public_definitions)

foreach(_feature_name IN LISTS _ekf2_features)
	if(PX4_EKF2_ENABLE_${_feature_name})
		set(CONFIG_EKF2_${_feature_name} ON)
		add_compile_definitions(CONFIG_EKF2_${_feature_name})
		list(APPEND _ekf2_public_definitions CONFIG_EKF2_${_feature_name})
	endif()
endforeach()

if(PX4_EKF2_ENABLE_GNSS AND PX4_EKF2_ENABLE_GNSS_YAW)
	set(CONFIG_EKF2_GNSS_YAW ON)
	add_compile_definitions(CONFIG_EKF2_GNSS_YAW)
	list(APPEND _ekf2_public_definitions CONFIG_EKF2_GNSS_YAW)
endif()

set(EKF_GENERATED_DERIVATION_INCLUDE_PATH "${PX4_EKF2_GENERATED_INCLUDE_DIR}")
include_directories(BEFORE
	"${PX4_mathlib_DIR}"
	"${PX4_matrix_DIR}"
	"${CMAKE_SOURCE_DIR}/PX4_geo"
	"${CMAKE_SOURCE_DIR}/PX4_lat_lon_alt"
	"${CMAKE_SOURCE_DIR}/PX4_atmosphere"
	"${CMAKE_SOURCE_DIR}/PX4_world_magnetic_model"
	"${CMAKE_CURRENT_SOURCE_DIR}/EKF/ringbuffer"
	"${CMAKE_CURRENT_SOURCE_DIR}/EKF"
	"${PX4_EKF2_GENERATED_INCLUDE_DIR}"
)
add_subdirectory(EKF)

target_include_directories(ecl_EKF
	PUBLIC
		"${CMAKE_CURRENT_SOURCE_DIR}"
		"${PX4_mathlib_DIR}"
		"${PX4_matrix_DIR}"
		"${CMAKE_SOURCE_DIR}/PX4_geo"
		"${CMAKE_SOURCE_DIR}/PX4_lat_lon_alt"
		"${CMAKE_SOURCE_DIR}/PX4_atmosphere"
		"${CMAKE_SOURCE_DIR}/PX4_world_magnetic_model"
		"${CMAKE_CURRENT_SOURCE_DIR}/EKF/ringbuffer"
		"${CMAKE_CURRENT_SOURCE_DIR}/EKF"
)

target_compile_features(ecl_EKF PUBLIC cxx_std_17)
target_compile_definitions(ecl_EKF PUBLIC ${_ekf2_public_definitions})

target_link_libraries(ecl_EKF
	PUBLIC
		PX4::mathlib
		PX4::matrix
		PX4::geo
		PX4::lat_lon_alt
		PX4::atmosphere
		PX4::world_magnetic_model
)

add_library(PX4::ekf2_core ALIAS ecl_EKF)
add_library(PX4::ekf2 ALIAS ecl_EKF)
target_sources(ecl_EKF PRIVATE EKF2.cpp EKF2Selector.cpp)

option(PX4_EKF2_BUILD_EXAMPLE "Build the host-side EKF2 example" OFF)
if(PX4_EKF2_BUILD_EXAMPLE)
	add_executable(ekf2_standalone_example examples/standalone_ekf2.cpp)
	target_link_libraries(ekf2_standalone_example PRIVATE PX4::ekf2)
endif()
