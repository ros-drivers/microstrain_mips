/**
 * @file    microstrain_3dm_gx5_45.h
 * @author  Brian S. Bingham
 * @version 0.1
 *
 * @brief ROS Node for Microstrain
 *
 */


#ifndef _MICROSTRAIN_3DM_GX5_45_H
#define _MICROSTRAIN_3DM_GX5_45_H

// Tell compiler that the following MIP SDI are C functions
extern "C" {
#include "mip_sdk.h"
#include "byteswap_utilities.h"
#include "mip_gx4_imu.h"
#include "mip_gx4_45.h"
}

#include <cstdio>
#include <unistd.h>


// ROS
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_srvs/Empty.h"

#define MIP_SDK_GX4_45_IMU_STANDARD_MODE	0x01
#define MIP_SDK_GX4_45_IMU_DIRECT_MODE	0x02

#define NUM_COMMAND_LINE_ARGUMENTS 3

#define DEFAULT_PACKET_TIMEOUT_MS  1000 //milliseconds

//macro to cause Sleep call to behave as it does for windows
#define Sleep(x) usleep(x*1000.0)


namespace Microstrain
{
  class Microstrain
  {
  public:
    //! @brief Constructor
    //!
    Microstrain();

    //! @brief Destructor
    //!
    ~Microstrain();

    //! @brief Main run loop
    //!
    void run();
   
    //! @brief Nav estimate callback
    void filter_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
    //! @brief AHRS callback
    void ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
    //! @brief GPS callback
    void gps_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
    
  private:
  //! @brief Reset KF service callback
  bool reset_callback(std_srvs::Empty::Request &req,
		      std_srvs::Empty::Response &resp);
  //! @brief Convience for printing packet stats
  void print_packet_stats();

  // Variables/fields
  u8 enable_data_stats_output_;

  //The primary device interface structure
  mip_interface device_interface_;

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
  sensor_msgs::NavSatFix gps_msg_;
  sensor_msgs::Imu imu_msg_;
  nav_msgs::Odometry nav_msg_;
  std_msgs::Int16MultiArray nav_status_msg_;
  std::string gps_frame_id_;
  std::string imu_frame_id_;
  std::string nav_frame_id_;
    
  // Update rates
  int nav_rate_;
  int imu_rate_;
  int gps_rate_;
  }; // Microstrain class

  
  // Define wrapper functions that call the Microstrain member functions
#ifdef __cplusplus
  extern "C"
#endif
  {
    
    //! @brief Nav filter callback wrapper
    void filter_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
    void ahrs_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
    void gps_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
    
#ifdef __cplusplus
  }
#endif
  
} // namespace Microstrain

#endif  // _MICROSTRAIN_3DM_GX5_45_H
