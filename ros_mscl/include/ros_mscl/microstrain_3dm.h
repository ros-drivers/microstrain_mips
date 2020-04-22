/*

Copyright (c) 2017, Brian Bingham
Copyright (c)  2020, Parker Hannifin Corp

This code is licensed under MIT license (see LICENSE file for details)

*/


#ifndef _MICROSTRAIN_3DM_H
#define _MICROSTRAIN_3DM_H


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
#include "std_srvs/Trigger.h"
#include "std_msgs/String.h"
#include "ros_mscl/status_msg.h"

#include "mscl/mscl.h"

#include "ros_mscl/SetAccelBias.h"
#include "ros_mscl/SetGyroBias.h"
#include "ros_mscl/SetHardIronValues.h"
#include "ros_mscl/SetSoftIronMatrix.h"
#include "ros_mscl/SetComplementaryFilter.h"
#include "ros_mscl/SetFilterEuler.h"
#include "ros_mscl/SetFilterHeading.h"
#include "ros_mscl/SetAccelBiasModel.h"
#include "ros_mscl/SetAccelAdaptiveVals.h"
#include "ros_mscl/SetSensorVehicleFrameTrans.h"
#include "ros_mscl/SetSensorVehicleFrameOffset.h"
#include "ros_mscl/SetReferencePosition.h"
#include "ros_mscl/SetConingScullingComp.h"
#include "ros_mscl/SetEstimationControlFlags.h"
#include "ros_mscl/SetDynamicsMode.h"
#include "ros_mscl/SetZeroAngleUpdateThreshold.h"
#include "ros_mscl/SetTareOrientation.h"
#include "ros_mscl/SetAccelNoise.h"
#include "ros_mscl/SetGyroNoise.h"
#include "ros_mscl/SetMagNoise.h"
#include "ros_mscl/SetGyroBiasModel.h"
#include "ros_mscl/SetMagAdaptiveVals.h"
#include "ros_mscl/SetMagDipAdaptiveVals.h"


#define MIP_SDK_GX4_45_IMU_STANDARD_MODE  0x01
#define MIP_SDK_GX4_45_IMU_DIRECT_MODE  0x02

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

    void parseMipPacket(const mscl::MipDataPacket& packet);
    void parseSensorPacket(const mscl::MipDataPacket& packet);
    void parseEstFilterPacket(const mscl::MipDataPacket& packet);
    void parseGnssPacket(const mscl::MipDataPacket& packet);

    void device_status_callback();

    bool set_accel_bias(ros_mscl::SetAccelBias::Request &req, ros_mscl::SetAccelBias::Response &res);

    bool get_accel_bias(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool set_gyro_bias(ros_mscl::SetGyroBias::Request &req, ros_mscl::SetGyroBias::Response &res);

    bool get_gyro_bias(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool set_hard_iron_values(ros_mscl::SetHardIronValues::Request &req, ros_mscl::SetHardIronValues::Response &res);

    bool get_hard_iron_values(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool device_report(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool gyro_bias_capture(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool set_soft_iron_matrix(ros_mscl::SetSoftIronMatrix::Request &req, ros_mscl::SetSoftIronMatrix::Response &res);

    bool get_soft_iron_matrix(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool set_complementary_filter(ros_mscl::SetComplementaryFilter::Request &req, ros_mscl::SetComplementaryFilter::Response &res);

    bool get_complementary_filter(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool set_filter_euler(ros_mscl::SetFilterEuler::Request &req, ros_mscl::SetFilterEuler::Response &res);

    bool set_filter_heading(ros_mscl::SetFilterHeading::Request &req, ros_mscl::SetFilterHeading::Response &res);

    bool set_accel_bias_model(ros_mscl::SetAccelBiasModel::Request &req, ros_mscl::SetAccelBiasModel::Response &res);

    bool set_accel_adaptive_vals(ros_mscl::SetAccelAdaptiveVals::Request &req, ros_mscl::SetAccelAdaptiveVals::Response &res);

    bool set_sensor_vehicle_frame_trans(ros_mscl::SetSensorVehicleFrameTrans::Request &req, ros_mscl::SetSensorVehicleFrameTrans::Response &res);

    bool get_sensor_vehicle_frame_trans(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool set_sensor_vehicle_frame_offset(ros_mscl::SetSensorVehicleFrameOffset::Request &req, ros_mscl::SetSensorVehicleFrameOffset::Response &res);

    bool get_sensor_vehicle_frame_offset(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool set_reference_position(ros_mscl::SetReferencePosition::Request &req, ros_mscl::SetReferencePosition::Response &res);

    bool get_reference_position(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool set_coning_sculling_comp(ros_mscl::SetConingScullingComp::Request &req, ros_mscl::SetConingScullingComp::Response &res);

    bool get_coning_sculling_comp(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool set_estimation_control_flags(ros_mscl::SetEstimationControlFlags::Request &req, ros_mscl::SetEstimationControlFlags::Response &res);

    bool get_estimation_control_flags(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool set_dynamics_mode(ros_mscl::SetDynamicsMode::Request &req, ros_mscl::SetDynamicsMode::Response &res);

    bool get_dynamics_mode(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool get_basic_status(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool get_diagnostic_report(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool set_zero_angle_update_threshold(ros_mscl::SetZeroAngleUpdateThreshold::Request &req, ros_mscl::SetZeroAngleUpdateThreshold::Response &res);

    bool get_zero_angle_update_threshold(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool set_tare_orientation(ros_mscl::SetTareOrientation::Request &req, ros_mscl::SetTareOrientation::Response &res);

    bool set_accel_noise(ros_mscl::SetAccelNoise::Request &req, ros_mscl::SetAccelNoise::Response &res);

    bool get_accel_noise(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool set_gyro_noise(ros_mscl::SetGyroNoise::Request &req, ros_mscl::SetGyroNoise::Response &res);

    bool get_gyro_noise(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool set_mag_noise(ros_mscl::SetMagNoise::Request &req, ros_mscl::SetMagNoise::Response &res);

    bool get_mag_noise(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool set_gyro_bias_model(ros_mscl::SetGyroBiasModel::Request &req, ros_mscl::SetGyroBiasModel::Response &res);

    bool get_gyro_bias_model(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool get_accel_adaptive_vals(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res );

    bool set_mag_adaptive_vals(ros_mscl::SetMagAdaptiveVals::Request &req, ros_mscl::SetMagAdaptiveVals::Response &res );

    bool get_mag_adaptive_vals(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res );

    bool set_mag_dip_adaptive_vals(ros_mscl::SetMagDipAdaptiveVals::Request &req, ros_mscl::SetMagDipAdaptiveVals::Response &res );

    bool get_mag_dip_adaptive_vals(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res );

    bool get_accel_bias_model(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

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
  mscl::InertialNode *msclInertialNode;

  //Packet Counters (valid, timeout, and checksum errors)
  uint32_t filter_valid_packet_count_;
  uint32_t ahrs_valid_packet_count_;
  uint32_t gps_valid_packet_count_;

  uint32_t filter_timeout_packet_count_;
  uint32_t ahrs_timeout_packet_count_;
  uint32_t gps_timeout_packet_count_;

  uint32_t filter_checksum_error_packet_count_;
  uint32_t ahrs_checksum_error_packet_count_;
  uint32_t gps_checksum_error_packet_count_;

  //Data field storage
  //AHRS
  float curr_ahrs_mag_x;
  float curr_ahrs_mag_y;
  float curr_ahrs_mag_z;

  mscl::Vector curr_ahrs_quaternion_;

  //FILTER
  mscl::Vector curr_filter_quaternion_;
  float curr_filter_velNorth;
  float curr_filter_velEast;
  float curr_filter_velDown;
  float curr_filter_posLat;
  float curr_filter_posLong;
  float curr_filter_posHeight;
  float curr_filter_roll;
  float curr_filter_pitch;
  float curr_filter_yaw;
  float curr_filter_angularRate_x;
  float curr_filter_angularRate_y;
  float curr_filter_angularRate_z;

  float curr_filter_pos_uncert_north;
  float curr_filter_pos_uncert_east;
  float curr_filter_pos_uncert_down;
  float curr_filter_vel_uncert_north;
  float curr_filter_vel_uncert_east;
  float curr_filter_vel_uncert_down;
  float curr_filter_att_uncert_roll;
  float curr_filter_att_uncert_pitch;
  float curr_filter_att_uncert_yaw;

  // ROS
  ros::Publisher gps_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher filtered_imu_pub_;
  ros::Publisher nav_pub_;
  ros::Publisher nav_status_pub_;
  ros::Publisher bias_pub_;
  ros::Publisher device_status_pub_;
  sensor_msgs::NavSatFix gps_msg_;
  sensor_msgs::Imu imu_msg_;
  sensor_msgs::Imu filtered_imu_msg_;
  nav_msgs::Odometry nav_msg_;
  std_msgs::Int16MultiArray nav_status_msg_;
  geometry_msgs::Vector3 bias_msg_;
  std::string gps_frame_id_;
  std::string imu_frame_id_;
  std::string odom_frame_id_;
  std::string odom_child_frame_id_;
  ros_mscl::status_msg device_status_msg_;
  bool publish_gps_;
  bool publish_imu_;
  bool publish_odom_;
  bool publish_bias_;
  std::vector<double> imu_linear_cov_;
  std::vector<double> imu_angular_cov_;
  std::vector<double> imu_orientation_cov_;

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
  uint8_t  com_mode;
  double reference_position_command[3];
  double reference_position_readback[3];
  uint8_t dynamics_mode;
  }; // Microstrain class


  // Define wrapper functions that call the Microstrain member functions
#ifdef __cplusplus
  extern "C"
#endif
  {

#ifdef __cplusplus
  }
#endif

} // namespace Microstrain

#endif  // _MICROSTRAIN_3DM_GX5_45_H
