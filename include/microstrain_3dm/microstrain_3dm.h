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


#ifndef _MICROSTRAIN_3DM_H
#define _MICROSTRAIN_3DM_H

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
#include "std_msgs/String.h"
#include "microstrain_3dm/status_msg.h"

#include "microstrain_3dm/SetAccelBias.h"
#include "microstrain_3dm/GetAccelBias.h"
#include "microstrain_3dm/GetGyroBias.h"
#include "microstrain_3dm/GetHardIronValues.h"
#include "microstrain_3dm/GetSoftIronMatrix.h"
#include "microstrain_3dm/SetGyroBias.h"
#include "microstrain_3dm/SetHardIronValues.h"
#include "microstrain_3dm/DeviceReport.h"
#include "microstrain_3dm/GyroBiasCapture.h"
#include "microstrain_3dm/SetSoftIronMatrix.h"
#include "microstrain_3dm/SetComplementaryFilter.h"
#include "microstrain_3dm/SetFilterEuler.h"
#include "microstrain_3dm/SetFilterHeading.h"
#include "microstrain_3dm/SetAccelBiasModel.h"
#include "microstrain_3dm/SetAccelAdaptiveVals.h"
#include "microstrain_3dm/SetSensorVehicleFrameTrans.h"
#include "microstrain_3dm/SetSensorVehicleFrameOffset.h"
#include "microstrain_3dm/GetSensorVehicleFrameOffset.h"
#include "microstrain_3dm/GetSensorVehicleFrameTrans.h"
#include "microstrain_3dm/GetComplementaryFilter.h"
#include "microstrain_3dm/SetReferencePosition.h"
#include "microstrain_3dm/GetReferencePosition.h"
#include "microstrain_3dm/SetConingScullingComp.h"
#include "microstrain_3dm/GetConingScullingComp.h"
#include "microstrain_3dm/SetEstimationControlFlags.h"
#include "microstrain_3dm/GetEstimationControlFlags.h"
#include "microstrain_3dm/SetDynamicsMode.h"
#include "microstrain_3dm/GetDynamicsMode.h"
#include "microstrain_3dm/GetBasicStatus.h"
#include "microstrain_3dm/GetDiagnosticReport.h"
#include "microstrain_3dm/SetZeroAngleUpdateThreshold.h"
#include "microstrain_3dm/GetZeroAngleUpdateThreshold.h"
#include "microstrain_3dm/SetTareOrientation.h"
#include "microstrain_3dm/SetAccelNoise.h"
#include "microstrain_3dm/GetAccelNoise.h"
#include "microstrain_3dm/SetGyroNoise.h"
#include "microstrain_3dm/GetGyroNoise.h"
#include "microstrain_3dm/SetMagNoise.h"
#include "microstrain_3dm/GetMagNoise.h"
#include "microstrain_3dm/SetGyroBiasModel.h"
#include "microstrain_3dm/GetGyroBiasModel.h"
#include "microstrain_3dm/GetAccelAdaptiveVals.h"
#include "microstrain_3dm/SetMagAdaptiveVals.h"
#include "microstrain_3dm/GetMagAdaptiveVals.h"
#include "microstrain_3dm/SetMagDipAdaptiveVals.h"
#include "microstrain_3dm/GetMagDipAdaptiveVals.h"
#include "microstrain_3dm/GetAccelBiasModel.h"


#define MIP_SDK_GX4_45_IMU_STANDARD_MODE	0x01
#define MIP_SDK_GX4_45_IMU_DIRECT_MODE	0x02

#define NUM_COMMAND_LINE_ARGUMENTS 3

#define DEFAULT_PACKET_TIMEOUT_MS  1000 //milliseconds

//macro to cause Sleep call to behave as it does for windows
#define Sleep(x) usleep(x*1000.0)

#define GX5_45_DEVICE "3DM-GX5-45"
#define GX5_35_DEVICE "3DM-GX5-35"
#define GX5_25_DEVICE "3DM-GX5-25"
#define GX5_15_DEVICE "3DM-GX5-15"



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

    void device_status_callback();

    bool set_accel_bias(microstrain_3dm::SetAccelBias::Request &req, microstrain_3dm::SetAccelBias::Response &res);

    bool get_accel_bias(microstrain_3dm::GetAccelBias::Request &req, microstrain_3dm::GetAccelBias::Response &res);

    bool set_gyro_bias(microstrain_3dm::SetGyroBias::Request &req, microstrain_3dm::SetGyroBias::Response &res);

    bool get_gyro_bias(microstrain_3dm::GetGyroBias::Request &req, microstrain_3dm::GetGyroBias::Response &res);

    bool set_hard_iron_values(microstrain_3dm::SetHardIronValues::Request &req, microstrain_3dm::SetHardIronValues::Response &res);

    bool get_hard_iron_values(microstrain_3dm::GetHardIronValues::Request &req, microstrain_3dm::GetHardIronValues::Response &res);

    bool device_report(microstrain_3dm::DeviceReport::Request &req, microstrain_3dm::DeviceReport::Response &res);

    bool gyro_bias_capture(microstrain_3dm::GyroBiasCapture::Request &req, microstrain_3dm::GyroBiasCapture::Response &res);

    bool set_soft_iron_matrix(microstrain_3dm::SetSoftIronMatrix::Request &req, microstrain_3dm::SetSoftIronMatrix::Response &res);

    bool get_soft_iron_matrix(microstrain_3dm::GetSoftIronMatrix::Request &req, microstrain_3dm::GetSoftIronMatrix::Response &res);

    bool set_complementary_filter(microstrain_3dm::SetComplementaryFilter::Request &req, microstrain_3dm::SetComplementaryFilter::Response &res);

    bool get_complementary_filter(microstrain_3dm::GetComplementaryFilter::Request &req, microstrain_3dm::GetComplementaryFilter::Response &res);

    bool set_filter_euler(microstrain_3dm::SetFilterEuler::Request &req, microstrain_3dm::SetFilterEuler::Response &res);

    bool set_filter_heading(microstrain_3dm::SetFilterHeading::Request &req, microstrain_3dm::SetFilterHeading::Response &res);

    bool set_accel_bias_model(microstrain_3dm::SetAccelBiasModel::Request &req, microstrain_3dm::SetAccelBiasModel::Response &res);

    bool set_accel_adaptive_vals(microstrain_3dm::SetAccelAdaptiveVals::Request &req, microstrain_3dm::SetAccelAdaptiveVals::Response &res);

    bool set_sensor_vehicle_frame_trans(microstrain_3dm::SetSensorVehicleFrameTrans::Request &req, microstrain_3dm::SetSensorVehicleFrameTrans::Response &res);

    bool get_sensor_vehicle_frame_trans(microstrain_3dm::GetSensorVehicleFrameTrans::Request &req, microstrain_3dm::GetSensorVehicleFrameTrans::Response &res);

    bool set_sensor_vehicle_frame_offset(microstrain_3dm::SetSensorVehicleFrameOffset::Request &req, microstrain_3dm::SetSensorVehicleFrameOffset::Response &res);

    bool get_sensor_vehicle_frame_offset(microstrain_3dm::GetSensorVehicleFrameOffset::Request &req, microstrain_3dm::GetSensorVehicleFrameOffset::Response &res);

    bool set_reference_position(microstrain_3dm::SetReferencePosition::Request &req, microstrain_3dm::SetReferencePosition::Response &res);

    bool get_reference_position(microstrain_3dm::GetReferencePosition::Request &req, microstrain_3dm::GetReferencePosition::Response &res);

    bool set_coning_sculling_comp(microstrain_3dm::SetConingScullingComp::Request &req, microstrain_3dm::SetConingScullingComp::Response &res);

    bool get_coning_sculling_comp(microstrain_3dm::GetConingScullingComp::Request &req, microstrain_3dm::GetConingScullingComp::Response &res);

    bool set_estimation_control_flags(microstrain_3dm::SetEstimationControlFlags::Request &req, microstrain_3dm::SetEstimationControlFlags::Response &res);

    bool get_estimation_control_flags(microstrain_3dm::GetEstimationControlFlags::Request &req, microstrain_3dm::GetEstimationControlFlags::Response &res);

    bool set_dynamics_mode(microstrain_3dm::SetDynamicsMode::Request &req, microstrain_3dm::SetDynamicsMode::Response &res);

    bool get_dynamics_mode(microstrain_3dm::GetDynamicsMode::Request &req, microstrain_3dm::GetDynamicsMode::Response &res);

    bool get_basic_status(microstrain_3dm::GetBasicStatus::Request &req, microstrain_3dm::GetBasicStatus::Response &res);

    u16 mip_3dm_cmd_hw_specific_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer);

    bool get_diagnostic_report(microstrain_3dm::GetDiagnosticReport::Request &req, microstrain_3dm::GetDiagnosticReport::Response &res);

    bool set_zero_angle_update_threshold(microstrain_3dm::SetZeroAngleUpdateThreshold::Request &req, microstrain_3dm::SetZeroAngleUpdateThreshold::Response &res);

    bool get_zero_angle_update_threshold(microstrain_3dm::GetZeroAngleUpdateThreshold::Request &req, microstrain_3dm::GetZeroAngleUpdateThreshold::Response &res);

    bool set_tare_orientation(microstrain_3dm::SetTareOrientation::Request &req, microstrain_3dm::SetTareOrientation::Response &res);

    bool set_accel_noise(microstrain_3dm::SetAccelNoise::Request &req, microstrain_3dm::SetAccelNoise::Response &res);

    bool get_accel_noise(microstrain_3dm::GetAccelNoise::Request &req, microstrain_3dm::GetAccelNoise::Response &res);

    bool set_gyro_noise(microstrain_3dm::SetGyroNoise::Request &req, microstrain_3dm::SetGyroNoise::Response &res);

    bool get_gyro_noise(microstrain_3dm::GetGyroNoise::Request &req, microstrain_3dm::GetGyroNoise::Response &res);

    bool set_mag_noise(microstrain_3dm::SetMagNoise::Request &req, microstrain_3dm::SetMagNoise::Response &res);

    bool get_mag_noise(microstrain_3dm::GetMagNoise::Request &req, microstrain_3dm::GetMagNoise::Response &res);

    bool set_gyro_bias_model(microstrain_3dm::SetGyroBiasModel::Request &req, microstrain_3dm::SetGyroBiasModel::Response &res);

    bool get_gyro_bias_model(microstrain_3dm::GetGyroBiasModel::Request &req, microstrain_3dm::GetGyroBiasModel::Response &res);

    bool get_accel_adaptive_vals(microstrain_3dm::GetAccelAdaptiveVals::Request &req, microstrain_3dm::GetAccelAdaptiveVals::Response &res );

    bool set_mag_adaptive_vals(microstrain_3dm::SetMagAdaptiveVals::Request &req, microstrain_3dm::SetMagAdaptiveVals::Response &res );

    bool get_mag_adaptive_vals(microstrain_3dm::GetMagAdaptiveVals::Request &req, microstrain_3dm::GetMagAdaptiveVals::Response &res );

    bool set_mag_dip_adaptive_vals(microstrain_3dm::SetMagDipAdaptiveVals::Request &req, microstrain_3dm::SetMagDipAdaptiveVals::Response &res );

    bool get_mag_dip_adaptive_vals(microstrain_3dm::GetMagDipAdaptiveVals::Request &req, microstrain_3dm::GetMagDipAdaptiveVals::Response &res );

    bool get_accel_bias_model(microstrain_3dm::GetAccelBiasModel::Request &req, microstrain_3dm::GetAccelBiasModel::Response &res);

    bool get_model_gps()
    {
      if (Microstrain::GX5_45 || Microstrain::GX5_35)
        return true;
      else
        return false;
    }


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
  ros::Publisher device_status_pub_;
  sensor_msgs::NavSatFix gps_msg_;
  sensor_msgs::Imu imu_msg_;
  nav_msgs::Odometry nav_msg_;
  std_msgs::Int16MultiArray nav_status_msg_;
  geometry_msgs::Vector3 bias_msg_;
  std::string gps_frame_id_;
  std::string imu_frame_id_;
  std::string odom_frame_id_;
  std::string odom_child_frame_id_;
  microstrain_3dm::status_msg device_status_msg_;
  bool publish_gps_;
  bool publish_imu_;
  bool publish_odom_;
  bool publish_bias_;

  //Device Flags
  bool GX5_15;
  bool GX5_25;
  bool GX5_35;
  bool GX5_45;
  bool GQX_45;
  bool RQX_45;
  bool CXX_45;
  bool CVX_10;
  bool CVX_15;
  bool CVX_25;





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
  gx4_25_diagnostic_device_status_field diagnostic_field;
  gx4_45_basic_status_field basic_field_45;
  gx4_45_diagnostic_device_status_field diagnostic_field_45;
  mip_complementary_filter_settings comp_filter_command, comp_filter_readback;
  mip_filter_accel_magnitude_error_adaptive_measurement_command accel_magnitude_error_command, accel_magnitude_error_readback;
  mip_filter_magnetometer_magnitude_error_adaptive_measurement_command mag_magnitude_error_command, mag_magnitude_error_readback;
  mip_filter_magnetometer_dip_angle_error_adaptive_measurement_command mag_dip_angle_error_command, mag_dip_angle_error_readback;
  mip_filter_zero_update_command zero_update_control, zero_update_readback;
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
