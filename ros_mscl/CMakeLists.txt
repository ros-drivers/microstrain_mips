cmake_minimum_required(VERSION 2.8.3)
project(ros_mscl)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslint
  message_generation
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  diagnostic_updater
)

find_library(MSCL_LIB_PATH NAMES libmscl.so PATHS "/usr/share/c++-mscl" DOC "MSCL Library" NO_DEFAULT_PATH)
find_library(BOOST_LIB_PATH NAMES libboost_chrono.so PATHS "/usr/share/c++-mscl/Boost/lib" NO_DEFAULT_PATH)
set(MSCL_INC_PATH "/usr/share/c++-mscl/source")
set(BOOST_INC_PATH "/usr/share/c++-mscl/Boost/include")
#find_path(MSCL_INC_PATH NAMES mscl.h PATHS "/usr/share/c++-mscl/source/mscl" DOC "MSCL Includes" NO_DEFAULT_PATH)

set(MSCL_LIB "usr/share/c++-mscl")
add_definitions(-std=c++11)

set(CMAKE_C_FLAGS "-Wno-implicit-function-declaration -Wno-incompatible-pointer-types -Wno-format -fno-builtin-memcpy")

add_message_files(
  FILES
  status_msg.msg
  filter_status_msg.msg
  filter_heading_msg.msg
  filter_heading_state_msg.msg
)

add_service_files(
  FILES
  InitFilterEuler.srv
  InitFilterHeading.srv
  DeviceSettings.srv
  SetAccelBias.srv
  GetAccelBias.srv
  SetGyroBias.srv
  GetGyroBias.srv
  SetHardIronValues.srv
  GetHardIronValues.srv
  SetSoftIronMatrix.srv
  GetSoftIronMatrix.srv
  SetComplementaryFilter.srv
  GetComplementaryFilter.srv
  SetAccelBiasModel.srv
  GetAccelBiasModel.srv
  SetGravityAdaptiveVals.srv
  GetGravityAdaptiveVals.srv
  SetSensor2VehicleRotation.srv
  GetSensor2VehicleRotation.srv
  SetSensor2VehicleOffset.srv
  GetSensor2VehicleOffset.srv
  SetReferencePosition.srv
  GetReferencePosition.srv
  SetConingScullingComp.srv
  GetConingScullingComp.srv
  SetEstimationControlFlags.srv
  GetEstimationControlFlags.srv
  SetDynamicsMode.srv
  GetDynamicsMode.srv
  SetZeroAngleUpdateThreshold.srv
  GetZeroAngleUpdateThreshold.srv
  SetZeroVelocityUpdateThreshold.srv
  GetZeroVelocityUpdateThreshold.srv
  SetTareOrientation.srv
  GetTareOrientation.srv
  SetAccelNoise.srv
  GetAccelNoise.srv
  SetGyroNoise.srv
  GetGyroNoise.srv
  SetMagNoise.srv
  GetMagNoise.srv
  SetGyroBiasModel.srv
  GetGyroBiasModel.srv
  SetAccelAdaptiveVals.srv
  GetAccelAdaptiveVals.srv
  SetMagAdaptiveVals.srv
  GetMagAdaptiveVals.srv
  SetMagDipAdaptiveVals.srv
  GetMagDipAdaptiveVals.srv
  SetHeadingSource.srv
  GetHeadingSource.srv
  GetSensor2VehicleTransformation.srv
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    cmake_modules
    tf2
    tf2_ros
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    nav_msgs
    message_runtime
)
###########
## Build ##
###########

include_directories(include/${PROJECT_NAME} ${MIPLIBINC} ${MIPUTILINC} ${MSCL_INC_PATH} ${MSCL_LIB} ${MSCL_INC} ${catkin_INCLUDE_DIRS})

add_library(ros_mscl
  src/microstrain_3dm.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(microstrain_diagnostic_updater src/microstrain_diagnostic_updater.cpp)
add_dependencies(microstrain_diagnostic_updater ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Executables

add_executable(ros_mscl_node
  src/microstrain_3dm_node.cpp
  src/microstrain_diagnostic_updater.cpp
)

add_dependencies(ros_mscl_node
  ros_mscl_gencpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
add_executable(set_accel_bias_client src/set_accel_bias_client.cpp)

add_executable(get_accel_bias_client src/get_accel_bias_client.cpp)

add_executable(set_gyro_bias_client src/set_gyro_bias_client.cpp)

add_executable(get_gyro_bias_client src/get_gyro_bias_client.cpp)

add_executable(set_hard_iron_values_client src/set_hard_iron_values_client.cpp)

add_executable(get_hard_iron_values_client src/get_hard_iron_values_client.cpp)

add_executable(device_report_client src/device_report_client.cpp)

add_executable(gyro_bias_capture_client src/gyro_bias_capture_client.cpp)

add_executable(set_soft_iron_matrix_client src/set_soft_iron_matrix_client.cpp)

add_executable(get_soft_iron_matrix_client src/get_soft_iron_matrix_client.cpp)

add_executable(set_complementary_filter_client src/set_complementary_filter_client.cpp)

add_executable(get_complementary_filter_client src/get_complementary_filter_client.cpp)

add_executable(init_filter_euler_client src/init_filter_euler_client.cpp)

add_executable(init_filter_heading_client src/init_filter_heading_client.cpp)

add_executable(device_settings_client src/device_settings_client.cpp)

add_executable(set_accel_bias_model_client src/set_accel_bias_model_client.cpp)

add_executable(set_gravity_adaptive_vals_client src/set_gravity_adaptive_vals_client.cpp)

add_executable(set_sensor2vehicle_rotation_client src/set_sensor2vehicle_rotation_client.cpp)

add_executable(get_sensor2vehicle_rotation_client src/get_sensor2vehicle_rotation_client.cpp)

add_executable(set_sensor2vehicle_offset_client src/set_sensor2vehicle_offset_client.cpp)

add_executable(get_sensor2vehicle_offset_client src/get_sensor2vehicle_offset_client.cpp)

add_executable(set_reference_position_client src/set_reference_position_client.cpp)

add_executable(get_reference_position_client src/get_reference_position_client.cpp)

add_executable(set_coning_sculling_comp_client src/set_coning_sculling_comp_client.cpp)

add_executable(get_coning_sculling_comp_client src/get_coning_sculling_comp_client.cpp)

add_executable(set_estimation_control_flags_client src/set_estimation_control_flags_client.cpp)

add_executable(get_estimation_control_flags_client src/get_estimation_control_flags_client.cpp)

add_executable(set_dynamics_mode_client src/set_dynamics_mode_client.cpp)

add_executable(get_dynamics_mode_client src/get_dynamics_mode_client.cpp)

add_executable(get_basic_status_client src/get_basic_status_client.cpp)

add_executable(get_diagnostic_report_client src/get_diagnostic_report_client.cpp)

add_executable(set_zero_angle_update_threshold_client src/set_zero_angle_update_threshold_client.cpp)

add_executable(get_zero_angle_update_threshold_client src/get_zero_angle_update_threshold_client.cpp)

add_executable(set_zero_velocity_update_threshold_client src/set_zero_velocity_update_threshold_client.cpp)

add_executable(get_zero_velocity_update_threshold_client src/get_zero_velocity_update_threshold_client.cpp)

add_executable(set_tare_orientation_client src/set_tare_orientation_client.cpp)

add_executable(set_accel_noise_client src/set_accel_noise_client.cpp)

add_executable(get_accel_noise_client src/get_accel_noise_client.cpp)

add_executable(set_gyro_noise_client src/set_gyro_noise_client.cpp)

add_executable(get_gyro_noise_client src/get_gyro_noise_client.cpp)

add_executable(set_mag_noise_client src/set_mag_noise_client.cpp)

add_executable(get_mag_noise_client src/get_mag_noise_client.cpp)

add_executable(set_gyro_bias_model_client src/set_gyro_bias_model_client.cpp)

add_executable(get_gyro_bias_model_client src/get_gyro_bias_model_client.cpp)

add_executable(get_gravity_adaptive_vals_client src/get_gravity_adaptive_vals_client.cpp)

add_executable(set_mag_adaptive_vals_client src/set_mag_adaptive_vals_client.cpp)

add_executable(get_mag_adaptive_vals_client src/get_mag_adaptive_vals_client.cpp)

add_executable(set_mag_dip_adaptive_vals_client src/set_mag_dip_adaptive_vals_client.cpp)

add_executable(get_mag_dip_adaptive_vals_client src/get_mag_dip_adaptive_vals_client.cpp)

add_executable(get_accel_bias_model_client src/get_accel_bias_model_client.cpp)

add_executable(commanded_vel_zupt_client src/commanded_vel_zupt_client.cpp)

add_executable(commanded_ang_rate_zupt_client src/commanded_ang_rate_zupt_client.cpp)

add_executable(set_heading_source_client src/set_heading_source_client.cpp)

add_dependencies(set_accel_bias_client ros_mscl_gencpp)

add_dependencies(set_accel_bias_client ros_mscl_gencpp)

add_dependencies(get_accel_bias_client ros_mscl_gencpp)

add_dependencies(set_gyro_bias_client ros_mscl_gencpp)

add_dependencies(get_gyro_bias_client ros_mscl_gencpp)

add_dependencies(set_hard_iron_values_client ros_mscl_gencpp)

add_dependencies(get_hard_iron_values_client ros_mscl_gencpp)

add_dependencies(device_report_client ros_mscl_gencpp)

add_dependencies(gyro_bias_capture_client ros_mscl_gencpp)

add_dependencies(set_soft_iron_matrix_client ros_mscl_gencpp)

add_dependencies(set_complementary_filter_client ros_mscl_gencpp)

add_dependencies(get_complementary_filter_client ros_mscl_gencpp)

add_dependencies(init_filter_euler_client ros_mscl_gencpp)

add_dependencies(init_filter_heading_client ros_mscl_gencpp)

add_dependencies(device_settings_client ros_mscl_gencpp)

add_dependencies(set_accel_bias_model_client ros_mscl_gencpp)

add_dependencies(set_gravity_adaptive_vals_client ros_mscl_gencpp)

add_dependencies(set_sensor2vehicle_rotation_client ros_mscl_gencpp)

add_dependencies(get_sensor2vehicle_rotation_client ros_mscl_gencpp)

add_dependencies(set_sensor2vehicle_offset_client ros_mscl_gencpp)

add_dependencies(get_sensor2vehicle_offset_client ros_mscl_gencpp)

add_dependencies(set_reference_position_client ros_mscl_gencpp)

add_dependencies(get_reference_position_client ros_mscl_gencpp)

add_dependencies(set_coning_sculling_comp_client ros_mscl_gencpp)

add_dependencies(get_coning_sculling_comp_client ros_mscl_gencpp)

add_dependencies(set_estimation_control_flags_client ros_mscl_gencpp)

add_dependencies(get_estimation_control_flags_client ros_mscl_gencpp)

add_dependencies(set_dynamics_mode_client ros_mscl_gencpp)

add_dependencies(get_dynamics_mode_client ros_mscl_gencpp)

add_dependencies(get_basic_status_client ros_mscl_gencpp)

add_dependencies(get_diagnostic_report_client ros_mscl_gencpp)

add_dependencies(set_zero_angle_update_threshold_client ros_mscl_gencpp)

add_dependencies(get_zero_angle_update_threshold_client ros_mscl_gencpp)

add_dependencies(set_zero_velocity_update_threshold_client ros_mscl_gencpp)

add_dependencies(get_zero_velocity_update_threshold_client ros_mscl_gencpp)

add_dependencies(set_tare_orientation_client ros_mscl_gencpp)

add_dependencies(set_accel_noise_client ros_mscl_gencpp)

add_dependencies(get_accel_noise_client ros_mscl_gencpp)

add_dependencies(set_gyro_noise_client ros_mscl_gencpp)

add_dependencies(get_gyro_noise_client ros_mscl_gencpp)

add_dependencies(set_mag_noise_client ros_mscl_gencpp)

add_dependencies(get_mag_noise_client ros_mscl_gencpp)

add_dependencies(set_gyro_bias_model_client ros_mscl_gencpp)

add_dependencies(get_gyro_bias_model_client ros_mscl_gencpp)

add_dependencies(get_gravity_adaptive_vals_client ros_mscl_gencpp)

add_dependencies(set_mag_adaptive_vals_client ros_mscl_gencpp)

add_dependencies(get_mag_adaptive_vals_client ros_mscl_gencpp)

add_dependencies(set_mag_dip_adaptive_vals_client ros_mscl_gencpp)

add_dependencies(get_mag_dip_adaptive_vals_client ros_mscl_gencpp)

add_dependencies(get_accel_bias_model_client ros_mscl_gencpp)

add_dependencies(commanded_vel_zupt_client ros_mscl_gencpp)

add_dependencies(commanded_ang_rate_zupt_client ros_mscl_gencpp)

add_dependencies(set_heading_source_client ros_mscl_gencpp)


# Linking

target_link_libraries(ros_mscl
  ${MSCL_LIB_PATH}
  ${catkin_LIBRARIES}
)
target_link_libraries(ros_mscl_node
  ros_mscl
  ${catkin_LIBRARIES}
)

target_link_libraries(commanded_vel_zupt_client ${catkin_LIBRARIES})

target_link_libraries(commanded_ang_rate_zupt_client ${catkin_LIBRARIES})

target_link_libraries(set_heading_source_client ${catkin_LIBRARIES})

target_link_libraries(set_accel_bias_client ${catkin_LIBRARIES})

target_link_libraries(get_accel_bias_client ${catkin_LIBRARIES})

target_link_libraries(set_gyro_bias_client ${catkin_LIBRARIES})

target_link_libraries(get_gyro_bias_client ${catkin_LIBRARIES})

target_link_libraries(set_hard_iron_values_client ${catkin_LIBRARIES})

target_link_libraries(get_hard_iron_values_client ${catkin_LIBRARIES})

target_link_libraries(device_report_client ${catkin_LIBRARIES})

target_link_libraries(gyro_bias_capture_client ${catkin_LIBRARIES})

target_link_libraries(set_soft_iron_matrix_client ${catkin_LIBRARIES})

target_link_libraries(get_soft_iron_matrix_client ${catkin_LIBRARIES})

target_link_libraries(set_complementary_filter_client ${catkin_LIBRARIES})

target_link_libraries(get_complementary_filter_client ${catkin_LIBRARIES})

target_link_libraries(init_filter_euler_client ${catkin_LIBRARIES})

target_link_libraries(init_filter_heading_client ${catkin_LIBRARIES})

target_link_libraries(device_settings_client ${catkin_LIBRARIES})

target_link_libraries(set_accel_bias_model_client ${catkin_LIBRARIES})

target_link_libraries(set_gravity_adaptive_vals_client ${catkin_LIBRARIES})

target_link_libraries(set_sensor2vehicle_rotation_client ${catkin_LIBRARIES})

target_link_libraries(get_sensor2vehicle_rotation_client ${catkin_LIBRARIES})

target_link_libraries(set_sensor2vehicle_offset_client ${catkin_LIBRARIES})

target_link_libraries(get_sensor2vehicle_offset_client ${catkin_LIBRARIES})

target_link_libraries(set_reference_position_client ${catkin_LIBRARIES})

target_link_libraries(get_reference_position_client ${catkin_LIBRARIES})

target_link_libraries(set_coning_sculling_comp_client ${catkin_LIBRARIES})

target_link_libraries(get_coning_sculling_comp_client ${catkin_LIBRARIES})

target_link_libraries(set_estimation_control_flags_client ${catkin_LIBRARIES})

target_link_libraries(get_estimation_control_flags_client ${catkin_LIBRARIES})

target_link_libraries(set_dynamics_mode_client ${catkin_LIBRARIES})

target_link_libraries(get_dynamics_mode_client ${catkin_LIBRARIES})

target_link_libraries(get_basic_status_client ${catkin_LIBRARIES})

target_link_libraries(get_diagnostic_report_client ${catkin_LIBRARIES})

target_link_libraries(set_zero_angle_update_threshold_client ${catkin_LIBRARIES})

target_link_libraries(get_zero_angle_update_threshold_client ${catkin_LIBRARIES})

target_link_libraries(set_zero_velocity_update_threshold_client ${catkin_LIBRARIES})

target_link_libraries(get_zero_velocity_update_threshold_client ${catkin_LIBRARIES})

target_link_libraries(set_tare_orientation_client ${catkin_LIBRARIES})

target_link_libraries(set_accel_noise_client ${catkin_LIBRARIES})

target_link_libraries(get_accel_noise_client ${catkin_LIBRARIES})

target_link_libraries(set_gyro_noise_client ${catkin_LIBRARIES})

target_link_libraries(get_gyro_noise_client ${catkin_LIBRARIES})

target_link_libraries(set_mag_noise_client ${catkin_LIBRARIES})

target_link_libraries(get_mag_noise_client ${catkin_LIBRARIES})

target_link_libraries(set_gyro_bias_model_client ${catkin_LIBRARIES})

target_link_libraries(get_gyro_bias_model_client ${catkin_LIBRARIES})

target_link_libraries(get_gravity_adaptive_vals_client ${catkin_LIBRARIES})

target_link_libraries(set_mag_adaptive_vals_client ${catkin_LIBRARIES})

target_link_libraries(get_mag_adaptive_vals_client ${catkin_LIBRARIES})

target_link_libraries(set_mag_dip_adaptive_vals_client ${catkin_LIBRARIES})

target_link_libraries(get_mag_dip_adaptive_vals_client ${catkin_LIBRARIES})

target_link_libraries(get_accel_bias_model_client ${catkin_LIBRARIES})



#############
## Install ##
#############

install(TARGETS ros_mscl ros_mscl_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Run roslint on only the node files for now
# TODO: Add more files as needed
roslint_cpp(src/microstrain_3dm.cpp src/microstrain_3dm_node.cpp)
