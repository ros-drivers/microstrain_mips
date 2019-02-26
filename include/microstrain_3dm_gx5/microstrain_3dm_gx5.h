/** ROS node

/*

Copyright (c) 2017, Brian Bingham
All rights reserved

This file is part of the microstrain_3dm_gx5_45 package.

microstrain_3dm_gx5_45 is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

microstrain_3dm_gx5_45 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

*/


#ifndef _MICROSTRAIN_3DM_GX5_H
#define _MICROSTRAIN_3DM_GX5_H

// Tell compiler that the following MIP SDI are C functions
extern "C" {
#include "mip_sdk.h"
#include "byteswap_utilities.h"
#include "mip_gx4_imu.h"
#include "mip_gx4_45.h"
#include "mip_gx4_25.h"
#include "mip_sdk_3dm.h"
#include "GX4-45_Test.h"
}

#include <cstdio>
#include <unistd.h>
#include <time.h>


// ROS
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_srvs/Empty.h"
#include "microstrain_3dm_gx5/SetAccelBias.h"
#include "microstrain_3dm_gx5/GetAccelBias.h"
#include "microstrain_3dm_gx5/GetGyroBias.h"
#include "microstrain_3dm_gx5/GetHardIronValues.h"
#include "microstrain_3dm_gx5/GetSoftIronMatrix.h"
#include "microstrain_3dm_gx5/SetGyroBias.h"
#include "microstrain_3dm_gx5/SetHardIronValues.h"
#include "microstrain_3dm_gx5/DeviceReport.h"
#include "microstrain_3dm_gx5/GyroBiasCapture.h"
#include "microstrain_3dm_gx5/SetSoftIronMatrix.h"
#include "microstrain_3dm_gx5/SetComplementaryFilter.h"
#include "microstrain_3dm_gx5/SetFilterEuler.h"
#include "microstrain_3dm_gx5/SetFilterHeading.h"
#include "microstrain_3dm_gx5/SetAccelBiasModel.h"
#include "microstrain_3dm_gx5/SetAccelAdaptiveVals.h"
#include "microstrain_3dm_gx5/SetSensorVehicleFrameTrans.h"
#include "microstrain_3dm_gx5/SetSensorVehicleFrameOffset.h"
#include "microstrain_3dm_gx5/GetSensorVehicleFrameTrans.h"
#include "microstrain_3dm_gx5/GetComplementaryFilter.h"
#include "microstrain_3dm_gx5/SetReferencePosition.h"
#include "microstrain_3dm_gx5/GetReferencePosition.h"
#include "microstrain_3dm_gx5/SetConingScullingComp.h"
#include "microstrain_3dm_gx5/GetConingScullingComp.h"
#include "microstrain_3dm_gx5/SetEstimationControlFlags.h"
#include "microstrain_3dm_gx5/GetEstimationControlFlags.h"
#include "microstrain_3dm_gx5/SetDynamicsMode.h"
#include "microstrain_3dm_gx5/GetBasicStatus.h"
#include "microstrain_3dm_gx5/GetDiagnosticReport.h"

#define MIP_SDK_GX4_45_IMU_STANDARD_MODE	0x01
#define MIP_SDK_GX4_45_IMU_DIRECT_MODE	0x02

#define NUM_COMMAND_LINE_ARGUMENTS 3

#define DEFAULT_PACKET_TIMEOUT_MS  1000 //milliseconds

//macro to cause Sleep call to behave as it does for windows
#define Sleep(x) usleep(x*1000.0)

/**
 * \brief Contains functions for micostrain driver
 */
namespace Microstrain
{
  /**
   * \brief Microstrain class
   *
   */
  class Microstrain
  {
  public:
    /**
     * Contructor
     */
    Microstrain();

    /** Destructor */
    ~Microstrain();

    /**
     * Main run loop
     */
    void run();

    //! Nav estimate callback
    void filter_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
    //! @brief AHRS callback
    void ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
    //! @brief GPS callback
    void gps_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);

    bool set_accel_bias(microstrain_3dm_gx5::SetAccelBias::Request &req, microstrain_3dm_gx5::SetAccelBias::Response &res);

    bool get_accel_bias(microstrain_3dm_gx5::GetAccelBias::Request &req, microstrain_3dm_gx5::GetAccelBias::Response &res);

    bool set_gyro_bias(microstrain_3dm_gx5::SetGyroBias::Request &req, microstrain_3dm_gx5::SetGyroBias::Response &res);

    bool get_gyro_bias(microstrain_3dm_gx5::GetGyroBias::Request &req, microstrain_3dm_gx5::GetGyroBias::Response &res);

    bool set_hard_iron_values(microstrain_3dm_gx5::SetHardIronValues::Request &req, microstrain_3dm_gx5::SetHardIronValues::Response &res);

    bool get_hard_iron_values(microstrain_3dm_gx5::GetHardIronValues::Request &req, microstrain_3dm_gx5::GetHardIronValues::Response &res);

    bool device_report(microstrain_3dm_gx5::DeviceReport::Request &req, microstrain_3dm_gx5::DeviceReport::Response &res);

    bool gyro_bias_capture(microstrain_3dm_gx5::GyroBiasCapture::Request &req, microstrain_3dm_gx5::GyroBiasCapture::Response &res);

    bool set_soft_iron_matrix(microstrain_3dm_gx5::SetSoftIronMatrix::Request &req, microstrain_3dm_gx5::SetSoftIronMatrix::Response &res);

    bool get_soft_iron_matrix(microstrain_3dm_gx5::GetSoftIronMatrix::Request &req, microstrain_3dm_gx5::GetSoftIronMatrix::Response &res);

    bool set_complementary_filter(microstrain_3dm_gx5::SetComplementaryFilter::Request &req, microstrain_3dm_gx5::SetComplementaryFilter::Response &res);

    bool get_complementary_filter(microstrain_3dm_gx5::GetComplementaryFilter::Request &req, microstrain_3dm_gx5::GetComplementaryFilter::Response &res);

    bool set_filter_euler(microstrain_3dm_gx5::SetFilterEuler::Request &req, microstrain_3dm_gx5::SetFilterEuler::Response &res);

    bool set_filter_heading(microstrain_3dm_gx5::SetFilterHeading::Request &req, microstrain_3dm_gx5::SetFilterHeading::Response &res);

    bool set_accel_bias_model(microstrain_3dm_gx5::SetAccelBiasModel::Request &req, microstrain_3dm_gx5::SetAccelBiasModel::Response &res);

    bool set_accel_adaptive_vals(microstrain_3dm_gx5::SetAccelAdaptiveVals::Request &req, microstrain_3dm_gx5::SetAccelAdaptiveVals::Response &res);

    bool set_sensor_vehicle_frame_trans(microstrain_3dm_gx5::SetSensorVehicleFrameTrans::Request &req, microstrain_3dm_gx5::SetSensorVehicleFrameTrans::Response &res);

    bool get_sensor_vehicle_frame_trans(microstrain_3dm_gx5::GetSensorVehicleFrameTrans::Request &req, microstrain_3dm_gx5::GetSensorVehicleFrameTrans::Response &res);

    bool set_sensor_vehicle_frame_offset(microstrain_3dm_gx5::SetSensorVehicleFrameOffset::Request &req, microstrain_3dm_gx5::SetSensorVehicleFrameOffset::Response &res);

    bool set_reference_position(microstrain_3dm_gx5::SetReferencePosition::Request &req, microstrain_3dm_gx5::SetReferencePosition::Response &res);

    bool get_reference_position(microstrain_3dm_gx5::GetReferencePosition::Request &req, microstrain_3dm_gx5::GetReferencePosition::Response &res);

    bool set_coning_sculling_comp(microstrain_3dm_gx5::SetConingScullingComp::Request &req, microstrain_3dm_gx5::SetConingScullingComp::Response &res);

    bool get_coning_sculling_comp(microstrain_3dm_gx5::GetConingScullingComp::Request &req, microstrain_3dm_gx5::GetConingScullingComp::Response &res);

    bool set_estimation_control_flags(microstrain_3dm_gx5::SetEstimationControlFlags::Request &req, microstrain_3dm_gx5::SetEstimationControlFlags::Response &res);

    bool get_estimation_control_flags(microstrain_3dm_gx5::GetEstimationControlFlags::Request &req, microstrain_3dm_gx5::GetEstimationControlFlags::Response &res);

    bool set_dynamics_mode(microstrain_3dm_gx5::SetDynamicsMode::Request &req, microstrain_3dm_gx5::SetDynamicsMode::Response &res);

    bool get_basic_status(microstrain_3dm_gx5::GetBasicStatus::Request &req, microstrain_3dm_gx5::GetBasicStatus::Response &res);

    u16 mip_3dm_cmd_hw_specific_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer);

    bool get_diagnostic_report(microstrain_3dm_gx5::GetDiagnosticReport::Request &req, microstrain_3dm_gx5::GetDiagnosticReport::Response &res);

  private:
  //! @brief Reset KF service callback
  bool reset_callback(std_srvs::Empty::Request &req,
		      std_srvs::Empty::Response &resp);
  //! @brief Convience for printing packet stats
  void print_packet_stats();



  // Variables/fields
  //The primary device interface structure
  mip_interface device_interface_;
  base_device_info_field device_info;
  //gx4_45_basic_status_field basic_field;
  u8  temp_string[20];

  //Packet Counters (valid, timeout, and checksum errors)
  u32 filter_valid_packet_count_;
  u32 ahrs_valid_packet_count_;
  u32 gps_valid_packet_count_;

  u32 filter_timeout_packet_count_;
  u32 ahrs_timeout_packet_count_;
  u32 gps_timeout_packet_count_;

  u32 filter_checksum_error_packet_count_;
  u32 ahrs_checksum_error_packet_count_;
  u32 gps_checksum_error_packet_count_;

  //Data field storage
  //AHRS
  mip_ahrs_scaled_gyro  curr_ahrs_gyro_;
  mip_ahrs_scaled_accel curr_ahrs_accel_;
  mip_ahrs_scaled_mag   curr_ahrs_mag_;
  mip_ahrs_quaternion  curr_ahrs_quaternion_;
  //GPS
  mip_gps_llh_pos curr_llh_pos_;
  mip_gps_ned_vel curr_ned_vel_;
  mip_gps_time    curr_gps_time_;

  //FILTER
  mip_filter_llh_pos               curr_filter_pos_;
  mip_filter_ned_velocity          curr_filter_vel_;
  mip_filter_attitude_euler_angles curr_filter_angles_;
  mip_filter_attitude_quaternion   curr_filter_quaternion_;
  mip_filter_compensated_angular_rate curr_filter_angular_rate_;
  mip_filter_llh_pos_uncertainty   curr_filter_pos_uncertainty_;
  mip_filter_ned_vel_uncertainty   curr_filter_vel_uncertainty_;
  mip_filter_euler_attitude_uncertainty curr_filter_att_uncertainty_;
  mip_filter_status curr_filter_status_;

  // ROS
  ros::Publisher gps_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher nav_pub_;
  ros::Publisher nav_status_pub_;
  ros::Publisher bias_pub_;
  sensor_msgs::NavSatFix gps_msg_;
  sensor_msgs::Imu imu_msg_;
  nav_msgs::Odometry nav_msg_;
  std_msgs::Int16MultiArray nav_status_msg_;
  geometry_msgs::Vector3 bias_msg_;
  std::string gps_frame_id_;
  std::string imu_frame_id_;
  std::string odom_frame_id_;
  std::string odom_child_frame_id_;
  bool publish_gps_;
  bool publish_imu_;
  bool publish_odom_;
  bool publish_bias_;

  // Update rates
  int nav_rate_;
  int imu_rate_;
  int gps_rate_;

  clock_t start;
  float field_data[3];
  float soft_iron[9];
  float soft_iron_readback[9];
  float angles[3];
  float heading_angle;
  float readback_angles[3];
  float noise[3];
  float beta[3];
  float readback_beta[3];
  float readback_noise[3];
  float offset[3];
  float readback_offset[3];
  u8  com_mode;
  u16 duration;
  u8 reference_position_enable_command;
  u8 reference_position_enable_readback;
  double reference_position_command[3];
  double reference_position_readback[3];
  u8 enable_flag;
  u16 estimation_control;
  u16 estimation_control_readback;
  u8 dynamics_mode;
  u8 readback_dynamics_mode;
  gx4_25_basic_status_field basic_field;
  gx4_imu_diagnostic_device_status_field imu_diagnostic_field;
  gx4_25_diagnostic_device_status_field diagnostic_field;
  mip_complementary_filter_settings comp_filter_command, comp_filter_readback;
  mip_filter_accel_magnitude_error_adaptive_measurement_command accel_magnitude_error_command, accel_magnitude_error_readback;
  }; // Microstrain class


  // Define wrapper functions that call the Microstrain member functions
#ifdef __cplusplus
  extern "C"
#endif
  {

    /**
     * Callback for KF estimate packets from sensor.
     */
    void filter_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
    /**
     * Callback for AHRS packets from sensor.
     */
    void ahrs_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
    /**
     * Callback for GPS packets from sensor.
     */
    void gps_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);

#ifdef __cplusplus
  }
#endif

} // namespace Microstrain

#endif  // _MICROSTRAIN_3DM_GX5_45_H
