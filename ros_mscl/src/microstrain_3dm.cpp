/*

Copyright (c) 2017, Brian Bingham
Copyright (c)  2020, Parker Hannifin Corp
This code is licensed under MIT license (see LICENSE file for details)

*/

#include <tf2/LinearMath/Transform.h>
#include <string>
#include <algorithm>
#include <time.h>

#include "mscl/mscl.h"
#include "ros_mscl/status_msg.h"
#include "microstrain_diagnostic_updater.h"
#include <vector>
#include <stdlib.h>

namespace Microstrain
{
Microstrain::Microstrain() : // Initialization list
                             filter_valid_packet_count_(0),
                             ahrs_valid_packet_count_(0),
                             gps_valid_packet_count_(0),
                             filter_timeout_packet_count_(0),
                             ahrs_timeout_packet_count_(0),
                             gps_timeout_packet_count_(0),
                             filter_checksum_error_packet_count_(0),
                             ahrs_checksum_error_packet_count_(0),
                             gps_checksum_error_packet_count_(0),
                             gps_frame_id_("gps_frame"),
                             imu_frame_id_("imu_frame"),
                             odom_frame_id_("odom_frame"),
                             odom_child_frame_id_("odom_frame"),
                             publish_gps_(true),
                             publish_imu_(true),
                             publish_odom_(true),
                             imu_linear_cov_(std::vector<double>(9, 0.0)),
                             imu_angular_cov_(std::vector<double>(9, 0.0)),
                             imu_orientation_cov_(std::vector<double>(9, 0.0))
{
  // pass
}
Microstrain::~Microstrain()
{
  // pass
}
void Microstrain::run()
{
  // Variables for device configuration, ROS parameters, etc.
  uint32_t com_port, baudrate;
  bool device_setup = false;
  bool readback_settings = true;
  bool save_settings = true;
  bool auto_init = true;
  int heading_source;
  int declination_source;
  uint8_t declination_source_u8;
  uint8_t heading_source_u8;
  uint8_t readback_declination_source;
  double declination;

  uint16_t duration = 0;

  com_mode = 0;

  // ROS setup
  ros::Time::init();
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  // ROS Parameters
  // Comms Parameters
  std::string port;
  int baud, pdyn_mode;
  private_nh.param("port", port, std::string("/dev/ttyACM1"));
  private_nh.param("baudrate", baud, 115200);
  baudrate = (uint32_t)baud;

  // Configuration Parameters
  private_nh.param("device_setup", device_setup, false);
  private_nh.param("readback_settings", readback_settings, true);
  private_nh.param("save_settings", save_settings, true);
  private_nh.param("heading_source", heading_source, 0x1);

  private_nh.param("auto_init", auto_init, true);
  private_nh.param("gps_rate", gps_rate_, 1);
  private_nh.param("imu_rate", imu_rate_, 10);
  private_nh.param("nav_rate", nav_rate_, 10);
  private_nh.param("dynamics_mode", pdyn_mode, 1);

  dynamics_mode = (uint8_t)pdyn_mode;
  if (dynamics_mode < 1 || dynamics_mode > 3)
  {
    ROS_WARN("dynamics_mode can't be %#04X, must be 1, 2 or 3.  Setting to 1.", dynamics_mode);
    dynamics_mode = 1;
  }

  private_nh.param("declination_source", declination_source, 2);
  if (declination_source < 1 || declination_source > 3)
  {
    ROS_WARN("declination_source can't be %#04X, must be 1, 2 or 3.  Setting to 2.", declination_source);
    declination_source = 2;
  }
  declination_source_u8 = (uint8_t)declination_source;
  heading_source_u8 = (uint8_t)heading_source;

  private_nh.param("declination", declination, 0.23);
  private_nh.param("gps_frame_id", gps_frame_id_, std::string("wgs84"));
  private_nh.param("imu_frame_id", imu_frame_id_, std::string("base_link"));
  private_nh.param("odom_frame_id", odom_frame_id_, std::string("wgs84"));
  private_nh.param("odom_child_frame_id", odom_child_frame_id_, std::string("base_link"));

  private_nh.param("publish_imu", publish_imu_, true);
  private_nh.param("publish_bias", publish_bias_, true);

  // Covariance parameters to set the sensor_msg/IMU covariance values
  std::vector<double> default_cov(9, 0.0);
  private_nh.param("imu_orientation_cov", imu_orientation_cov_, default_cov);
  private_nh.param("imu_linear_cov", imu_linear_cov_, default_cov);
  private_nh.param("imu_linear_cov", imu_linear_cov_, default_cov);
  private_nh.param("imu_angular_cov", imu_angular_cov_, default_cov);

  // ROS publishers and subscribers

  // Initialize the serial interface to the device

  try
  {
    ROS_INFO("Attempting to open serial port <%s> at <%d> \n", port.c_str(), baudrate);
    mscl::Connection connection = mscl::Connection::Serial(realpath(port.c_str(), NULL), baudrate);
    mscl::InertialNode inertialNode(connection);
    msclInertialNode = &inertialNode;

    const mscl::MipNodeFeatures &features = inertialNode.features();

    if (publish_imu_)
      imu_pub_ = node.advertise<sensor_msgs::Imu>("imu/data", 100);

    // Publishes device status
    device_status_pub_ = node.advertise<ros_mscl::status_msg>("device/status", 100);
    ros::ServiceServer get_basic_status_service = node.advertiseService("get_basic_status", &Microstrain::get_basic_status, this);
    ros::ServiceServer get_diagnostic_report_service = node.advertiseService("get_diagnostic_report", &Microstrain::get_diagnostic_report, this);
    ros::ServiceServer device_report_service = node.advertiseService("device_report", &Microstrain::device_report, this);

    // Services to set/get device functions
    ros::ServiceServer reset_filter = node.advertiseService("reset_kf", &Microstrain::reset_callback, this);

    ros::ServiceServer gyro_bias_capture_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_CAP_GYRO_BIAS))
    {
      gyro_bias_capture_service = node.advertiseService("gyro_bias_capture", &Microstrain::gyro_bias_capture, this);
    }

    ros::ServiceServer get_soft_iron_matrix_service;
    ros::ServiceServer set_soft_iron_matrix_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_MAG_SOFT_IRON_MATRIX))
    {
      set_soft_iron_matrix_service = node.advertiseService("set_soft_iron_matrix", &Microstrain::set_soft_iron_matrix, this);
      get_soft_iron_matrix_service = node.advertiseService("get_soft_iron_matrix", &Microstrain::get_soft_iron_matrix, this);
    }

    ros::ServiceServer set_complementary_filter_service;
    ros::ServiceServer get_complementary_filter_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_COMPLEMENTARY_FILTER_SETTINGS))
    {
      set_complementary_filter_service = node.advertiseService("set_complementary_filter", &Microstrain::set_complementary_filter, this);
      get_complementary_filter_service = node.advertiseService("get_complementary_filter", &Microstrain::get_complementary_filter, this);
    }

    ros::ServiceServer set_filter_euler_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_COMPLEMENTARY_FILTER_SETTINGS))
    {
      set_filter_euler_service = node.advertiseService("set_filter_euler", &Microstrain::set_filter_euler, this);
    }

    ros::ServiceServer set_filter_heading_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_EF_INIT_HEADING))
    {
      set_filter_heading_service = node.advertiseService("set_filter_heading", &Microstrain::set_filter_heading, this);
    }

    ros::ServiceServer set_accel_bias_model_service;
    ros::ServiceServer get_gyro_bias_model_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ACCEL_BIAS_MODEL_PARAMS))
    {
      set_accel_bias_model_service = node.advertiseService("set_accel_bias_model", &Microstrain::set_accel_bias_model, this);
      get_gyro_bias_model_service = node.advertiseService("get_gyro_bias_model", &Microstrain::get_gyro_bias_model, this);
    }

    ros::ServiceServer set_accel_adaptive_vals_service;
    ros::ServiceServer get_accel_adaptive_vals_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_EF_GRAV_MAGNITUDE_ERR_ADAPT_MEASURE))
    {
      set_accel_adaptive_vals_service = node.advertiseService("set_accel_adaptive_vals", &Microstrain::set_accel_adaptive_vals, this);
      get_accel_adaptive_vals_service = node.advertiseService("get_accel_adaptive_vals", &Microstrain::get_accel_adaptive_vals, this);
    }

    ros::ServiceServer set_sensor_vehicle_frame_trans_service;
    ros::ServiceServer get_sensor_vehicle_frame_trans_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_TRANS))
    {
      set_sensor_vehicle_frame_trans_service = node.advertiseService("set_sensor_vehicle_frame_trans", &Microstrain::set_sensor_vehicle_frame_trans, this);
      get_sensor_vehicle_frame_trans_service = node.advertiseService("get_sensor_vehicle_frame_trans", &Microstrain::get_sensor_vehicle_frame_trans, this);
    }

    ros::ServiceServer set_sensor_vehicle_frame_offset_service;
    ros::ServiceServer get_sensor_vehicle_frame_offset_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_OFFSET))
    {
      set_sensor_vehicle_frame_offset_service = node.advertiseService("set_sensor_vehicle_frame_offset", &Microstrain::set_sensor_vehicle_frame_offset, this);
      get_sensor_vehicle_frame_offset_service = node.advertiseService("get_sensor_vehicle_frame_offset", &Microstrain::get_sensor_vehicle_frame_offset, this);
    }

    ros::ServiceServer set_accel_bias_service;
    ros::ServiceServer get_accel_bias_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_ACCEL_BIAS))
    {
      set_accel_bias_service = node.advertiseService("set_accel_bias", &Microstrain::set_accel_bias, this);
      get_accel_bias_service = node.advertiseService("get_accel_bias", &Microstrain::get_accel_bias, this);
    }

    ros::ServiceServer set_gyro_bias_service;
    ros::ServiceServer get_gyro_bias_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_GYRO_BIAS))
    {
      set_gyro_bias_service = node.advertiseService("set_gyro_bias", &Microstrain::set_gyro_bias, this);
      get_gyro_bias_service = node.advertiseService("get_gyro_bias", &Microstrain::get_gyro_bias, this);
    }

    ros::ServiceServer set_hard_iron_values_service;
    ros::ServiceServer get_hard_iron_values_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_MAG_HARD_IRON_OFFSET))
    {
      set_hard_iron_values_service = node.advertiseService("set_hard_iron_values", &Microstrain::set_hard_iron_values, this);
      get_hard_iron_values_service = node.advertiseService("get_hard_iron_values", &Microstrain::get_hard_iron_values, this);
    }

    ros::ServiceServer set_reference_position_service;
    ros::ServiceServer get_reference_position_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SET_REF_POSITION))
    {
      set_reference_position_service = node.advertiseService("set_reference_position", &Microstrain::set_reference_position, this);
      get_reference_position_service = node.advertiseService("get_reference_position", &Microstrain::get_reference_position, this);
    }

    ros::ServiceServer set_coning_sculling_comp_service;
    ros::ServiceServer get_coning_sculling_comp_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_CONING_SCULLING))
    {
      set_coning_sculling_comp_service = node.advertiseService("set_coning_sculling_comp", &Microstrain::set_coning_sculling_comp, this);
      get_coning_sculling_comp_service = node.advertiseService("get_coning_sculling_comp", &Microstrain::get_coning_sculling_comp, this);
    }

    ros::ServiceServer set_estimation_control_flags_service;
    ros::ServiceServer get_estimation_control_flags_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_EF_BIAS_EST_CTRL))
    {
      set_estimation_control_flags_service = node.advertiseService("set_estimation_control_flags", &Microstrain::set_estimation_control_flags, this);
      get_estimation_control_flags_service = node.advertiseService("get_estimation_control_flags", &Microstrain::get_estimation_control_flags, this);
    }

    ros::ServiceServer set_dynamics_mode_service;
    ros::ServiceServer get_dynamics_mode_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_EF_VEHIC_DYNAMICS_MODE))
    {
      set_dynamics_mode_service = node.advertiseService("set_dynamics_mode", &Microstrain::set_dynamics_mode, this);
      get_dynamics_mode_service = node.advertiseService("get_dynamics_mode", &Microstrain::get_dynamics_mode, this);
    }

    ros::ServiceServer set_zero_angle_update_threshold_service;
    ros::ServiceServer get_zero_angle_update_threshold_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ZERO_ANG_RATE_UPDATE_CTRL))
    {
      set_zero_angle_update_threshold_service = node.advertiseService("set_zero_angle_update_threshold", &Microstrain::set_zero_angle_update_threshold, this);
      get_zero_angle_update_threshold_service = node.advertiseService("get_zero_angle_update_threshold", &Microstrain::get_zero_angle_update_threshold, this);
    }

    ros::ServiceServer set_tare_orientation_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_EF_TARE_ORIENT))
    {
      set_tare_orientation_service = node.advertiseService("set_tare_orientation", &Microstrain::set_tare_orientation, this);
    }

    ros::ServiceServer set_accel_noise_service;
    ros::ServiceServer get_accel_noise_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ACCEL_WHT_NSE_STD_DEV))
    {
      set_accel_noise_service = node.advertiseService("set_accel_noise", &Microstrain::set_accel_noise, this);
      get_accel_noise_service = node.advertiseService("get_accel_noise", &Microstrain::get_accel_noise, this);
    }

    ros::ServiceServer set_gyro_noise_service;
    ros::ServiceServer get_gyro_noise_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_EF_GYRO_WHT_NSE_STD_DEV))
    {
      set_gyro_noise_service = node.advertiseService("set_gyro_noise", &Microstrain::set_gyro_noise, this);
      get_gyro_noise_service = node.advertiseService("get_gyro_noise", &Microstrain::get_gyro_noise, this);
    }

    ros::ServiceServer set_mag_noise_service;
    ros::ServiceServer get_mag_noise_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_EF_HARD_IRON_OFFSET_PROCESS_NOISE))
    {
      set_mag_noise_service = node.advertiseService("set_mag_noise", &Microstrain::set_mag_noise, this);
      get_mag_noise_service = node.advertiseService("get_mag_noise", &Microstrain::get_mag_noise, this);
    }

    ros::ServiceServer set_mag_adaptive_vals_service;
    ros::ServiceServer get_mag_adaptive_vals_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_EF_MAG_MAGNITUDE_ERR_ADAPT_MEASURE))
    {
      set_mag_adaptive_vals_service = node.advertiseService("set_mag_adaptive_vals", &Microstrain::set_mag_adaptive_vals, this);
      get_mag_adaptive_vals_service = node.advertiseService("get_mag_adaptive_vals", &Microstrain::get_mag_adaptive_vals, this);
    }

    ros::ServiceServer set_mag_dip_adaptive_vals_service;
    ros::ServiceServer get_mag_dip_adaptive_vals_service;
    if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_EF_MAG_DIP_ANGLE_ERR_ADAPT_MEASURE))
    {
      set_mag_dip_adaptive_vals_service = node.advertiseService("set_mag_dip_adaptive_vals", &Microstrain::set_mag_dip_adaptive_vals, this);
      get_mag_dip_adaptive_vals_service = node.advertiseService("get_mag_dip_adaptive_vals", &Microstrain::get_mag_dip_adaptive_vals, this);
    }

    if (device_setup)
    {
      //print the device info
      ROS_INFO("Model Name: %s\n", inertialNode.modelName().c_str());
      ROS_INFO("Serial Number: %s\n", inertialNode.serialNumber().c_str());

      //enable publishing of fields depending on what the device supports
      bool supportsGNSS = inertialNode.features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS);
      bool supportsFilter = inertialNode.features().supportsCategory(mscl::MipTypes::DataClass::CLASS_ESTFILTER);

      private_nh.param("publish_gps", publish_gps_, supportsGNSS);
      private_nh.param("publish_odom", publish_odom_, supportsFilter);

      if (supportsGNSS)
      {
        gps_pub_ = node.advertise<sensor_msgs::NavSatFix>("gps/fix", 100);
      }

      if (supportsFilter)
      {
        nav_pub_ = node.advertise<nav_msgs::Odometry>("nav/odom", 100);
        nav_status_pub_ = node.advertise<std_msgs::Int16MultiArray>("nav/status", 100);
        filtered_imu_pub_ = node.advertise<sensor_msgs::Imu>("filtered/imu/data", 100);
      }

      // Put into idle mode
      ROS_INFO("Setting to Idle: Stopping data streams and/or waking from sleep");
      inertialNode.setToIdle();

      // AHRS Setup
      if (publish_imu_)
      {
        ROS_INFO("Setting AHRS active channel fields");
        mscl::SampleRate imuRate = mscl::SampleRate::Hertz(imu_rate_);

        mscl::MipTypes::MipChannelFields ahrsChannels{
            mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_ACCEL_VEC,
            mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_GYRO_VEC,
            mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_ORIENTATION_QUATERNION};

        mscl::MipChannels supportedChannels;
        for (mscl::MipTypes::ChannelField channel : msclInertialNode->features().supportedChannelFields(mscl::MipTypes::DataClass::CLASS_AHRS_IMU))
        {
          if (std::find(ahrsChannels.begin(), ahrsChannels.end(), channel) != ahrsChannels.end())
          {
            supportedChannels.push_back(mscl::MipChannel(channel, imuRate));
          }
        }

        inertialNode.setActiveChannelFields(mscl::MipTypes::DataClass::CLASS_AHRS_IMU, supportedChannels);

        if (features.supportsCommand(mscl::MipTypes::Command::CMD_EF_DECLINATION_SRC))
        {
          ROS_INFO("Setting Declination Source");
          inertialNode.setDeclinationSource(mscl::GeographicSourceOptions(static_cast<mscl::InertialTypes::GeographicSourceOption>(declination_source_u8), declination));
        }

        inertialNode.enableDataStream(mscl::MipTypes::DataClass::CLASS_AHRS_IMU);
      }

      //GNSS setup
      if (publish_gps_)
      {
        ROS_INFO("Setting GNSS active channel fields");
        mscl::SampleRate gpsRate = mscl::SampleRate::Hertz(gps_rate_);

        mscl::MipTypes::MipChannelFields gnssChannels{
            mscl::MipTypes::ChannelField::CH_FIELD_GNSS_LLH_POSITION,
            mscl::MipTypes::ChannelField::CH_FIELD_GNSS_NED_VELOCITY,
            mscl::MipTypes::ChannelField::CH_FIELD_GNSS_GPS_TIME};

        mscl::MipChannels supportedChannels;
        for (mscl::MipTypes::ChannelField channel : msclInertialNode->features().supportedChannelFields(mscl::MipTypes::DataClass::CLASS_GNSS))
        {
          if (std::find(gnssChannels.begin(), gnssChannels.end(), channel) != gnssChannels.end())
          {
            supportedChannels.push_back(mscl::MipChannel(channel, gpsRate));
          }
        }

        //set the GNSS channel fields
        inertialNode.setActiveChannelFields(mscl::MipTypes::DataClass::CLASS_GNSS, supportedChannels);

        inertialNode.enableDataStream(mscl::MipTypes::DataClass::CLASS_GNSS);
      }

      // Filter setup
      if (publish_odom_)
      {
        ROS_INFO("Setting Estimation Filter active channel fields");
        mscl::SampleRate navRate = mscl::SampleRate::Hertz(nav_rate_);

        mscl::MipTypes::MipChannelFields navChannels{
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_LLH_POS,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_NED_VELOCITY,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_LLH_UNCERT,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_NED_UNCERT,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ATT_UNCERT_QUAT,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_FILTER_STATUS};

        mscl::MipChannels supportedChannels;
        for (mscl::MipTypes::ChannelField channel : msclInertialNode->features().supportedChannelFields(mscl::MipTypes::DataClass::CLASS_ESTFILTER))
        {
          if (std::find(navChannels.begin(), navChannels.end(), channel) != navChannels.end())
          {
            supportedChannels.push_back(mscl::MipChannel(channel, navRate));
          }
        }

        inertialNode.setActiveChannelFields(mscl::MipTypes::DataClass::CLASS_ESTFILTER, supportedChannels);

        //set dynamics mode
        if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_EF_VEHIC_DYNAMICS_MODE))
        {
          mscl::VehicleModeTypes modes = inertialNode.features().supportedVehicleModeTypes();
          if (std::find(modes.begin(), modes.end(), static_cast<mscl::InertialTypes::VehicleModeType>(dynamics_mode)) != modes.end())
          {
            ROS_INFO("Setting dynamics mode to %#04X", static_cast<mscl::InertialTypes::VehicleModeType>(dynamics_mode));
            inertialNode.setVehicleDynamicsMode(static_cast<mscl::InertialTypes::VehicleModeType>(dynamics_mode));
          }
        }

        // Set heading Source
        if (inertialNode.features().supportsCommand(mscl::MipTypes::Command::CMD_EF_HEADING_UPDATE_CTRL))
        {
          for (mscl::HeadingUpdateOptions headingSources : inertialNode.features().supportedHeadingUpdateOptions())
          {
            if (headingSources.AsOptionId() == static_cast<mscl::InertialTypes::HeadingUpdateEnableOption>(heading_source))
            {
              ROS_INFO("Setting heading source to %#04X", heading_source);
              inertialNode.setHeadingUpdateControl(mscl::HeadingUpdateOptions(static_cast<mscl::InertialTypes::HeadingUpdateEnableOption>(heading_source)));
              break;
            }
          }
        }
        inertialNode.enableDataStream(mscl::MipTypes::DataClass::CLASS_ESTFILTER);
        inertialNode.setAutoInitialization(true);
        inertialNode.setInitialHeading(0);
      }

      if (save_settings)
      {
        //save the current settings as startup settings
        inertialNode.saveSettingsAsStartup();
      }
    }

    // Loop
    // Determine loop rate as 2*(max update rate), but abs. max of 1kHz
    int max_rate = std::max({publish_imu_ ? imu_rate_ : 1,
                             publish_gps_ ? gps_rate_ : 1,
                             publish_odom_ ? nav_rate_ : 1});

    int spin_rate = std::min(2 * max_rate, 1000);
    ROS_INFO("Setting spin rate to <%d>", spin_rate);
    ros::Rate r(spin_rate); // Rate in Hz

    ros_mscl::RosDiagnosticUpdater ros_diagnostic_updater(msclInertialNode);

    ROS_INFO("Starting Data Parsing");
    while (ros::ok())
    {
      mscl::MipDataPackets packets = inertialNode.getDataPackets(1000);

      for (mscl::MipDataPacket packet : packets)
      {
        parseMipPacket(packet);
      }
    }

    device_status_callback();
    ros::spinOnce(); // take care of service requests.
    r.sleep();
  }
  catch (mscl::Error_Connection)
  {
    ROS_ERROR("Device Disconnected");
  }

  catch (mscl::Error &e)
  {
    ROS_FATAL("Error: %s", e.what());
  }

  if (msclInertialNode)
  {
    msclInertialNode->connection().disconnect();
  }
} // End of ::run()

bool Microstrain::reset_callback(std_srvs::Empty::Request &req,
                                 std_srvs::Empty::Response &res)
{
  ROS_INFO("Resetting filter\n");
  if (msclInertialNode)
  {
    try
    {
      msclInertialNode->resetFilter();
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }
  return true;
}

// Services to get/set values on devices
// Set accel bias values
bool Microstrain::set_accel_bias(ros_mscl::SetAccelBias::Request &req,
                                 ros_mscl::SetAccelBias::Response &res)
{
  res.success = false;
  ROS_INFO("Setting accel bias values");
  if (msclInertialNode)
  {
    try
    {
      mscl::GeometricVector biasVector = msclInertialNode->getAccelerometerBias();

      ROS_INFO("Accel bias vector values are: %f %f %f",
               biasVector.x(), biasVector.y(), biasVector.z());
      ROS_INFO("Client request values are: %.2f %.2f %.2f",
               req.bias.x, req.bias.y, req.bias.z);

      biasVector.x(req.bias.x);
      biasVector.y(req.bias.y);
      biasVector.z(req.bias.z);

      msclInertialNode->setAccelerometerBias(biasVector);
      ROS_INFO("New accel bias vector values are: %.2f %.2f %.2f",
               biasVector.x(), biasVector.y(), biasVector.z());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }
  return res.success;
}

// Get accel bias values
bool Microstrain::get_accel_bias(std_srvs::Trigger::Request &req,
                                 std_srvs::Trigger::Response &res)
{
  res.success = false;
  ROS_INFO("Getting accel bias values\n");
  if (msclInertialNode)
  {
    try
    {
      mscl::GeometricVector biasVector = msclInertialNode->getAccelerometerBias();

      ROS_INFO("Accel bias vector values are: %f %f %f.\n",
               biasVector.x(), biasVector.y(), biasVector.z());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }
  return res.success;
}

// Set gyro bias values
bool Microstrain::set_gyro_bias(ros_mscl::SetGyroBias::Request &req,
                                ros_mscl::SetGyroBias::Response &res)
{
  res.success = false;
  ROS_INFO("Setting gyro bias values");
  if (msclInertialNode)
  {
    try
    {
      mscl::GeometricVector biasVector = msclInertialNode->getGyroBias();

      ROS_INFO("Gyro bias vector values are: %f %f %f",
               biasVector.x(), biasVector.y(), biasVector.z());
      ROS_INFO("Client request values are: %.2f %.2f %.2f",
               req.bias.x, req.bias.y, req.bias.z);

      biasVector.x(req.bias.x);
      biasVector.y(req.bias.y);
      biasVector.z(req.bias.z);

      msclInertialNode->setGyroBias(biasVector);

      ROS_INFO("New gyro bias vector values are: %.2f %.2f %.2f",
               biasVector.x(), biasVector.y(), biasVector.z());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get gyro bias values
bool Microstrain::get_gyro_bias(std_srvs::Trigger::Request &req,
                                std_srvs::Trigger::Response &res)
{
  res.success = false;
  ROS_INFO("Getting gyro bias values");
  if (msclInertialNode)
  {
    try
    {
      mscl::GeometricVector biasVector = msclInertialNode->getGyroBias();

      ROS_INFO("Gyro bias vector values are: %f %f %f",
               biasVector.x(), biasVector.y(), biasVector.z());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Set hard iron values
bool Microstrain::set_hard_iron_values(ros_mscl::SetHardIronValues::Request &req,
                                       ros_mscl::SetHardIronValues::Response &res)
{
  res.success = false;
  ROS_INFO("Setting hard iron values");
  if (msclInertialNode)
  {
    try
    {
      mscl::GeometricVector biasVector = msclInertialNode->getMagnetometerHardIronOffset();

      ROS_INFO("Hard Iron vector values are: %f %f %f",
               biasVector.x(), biasVector.y(), biasVector.z());
      ROS_INFO("Client request values are: %.2f %.2f %.2f",
               req.bias.x, req.bias.y, req.bias.z);

      biasVector.x(req.bias.x);
      biasVector.y(req.bias.y);
      biasVector.z(req.bias.z);

      msclInertialNode->setMagnetometerHardIronOffset(biasVector);

      ROS_INFO("New hard iron values are: %.2f %.2f %.2f",
               biasVector.x(), biasVector.y(), biasVector.z());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get hard iron values
bool Microstrain::get_hard_iron_values(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = false;
  ROS_INFO("Getting gyro bias values");
  if (msclInertialNode)
  {
    try
    {
      mscl::GeometricVector biasVector = msclInertialNode->getMagnetometerHardIronOffset();

      ROS_INFO("Hard iron values are: %f %f %f",
               biasVector.x(), biasVector.y(), biasVector.z());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get device report
bool Microstrain::device_report(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = false;

  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Model Name       => %s\n", msclInertialNode->modelName().c_str());
      ROS_INFO("Model Number     => %s\n", msclInertialNode->modelNumber().c_str());
      ROS_INFO("Serial Number    => %s\n", msclInertialNode->serialNumber().c_str());
      ROS_INFO("Options        => %s\n", msclInertialNode->deviceOptions().c_str());
      ROS_INFO("Firmware Version => %d.%d.%.2d\n\n", msclInertialNode->firmwareVersion().str());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Capture gyro bias values
bool Microstrain::gyro_bias_capture(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = false;
  ROS_INFO("Performing Gyro Bias capture.\nPlease keep device stationary during the 5 second gyro bias capture interval\n");

  if (msclInertialNode)
  {
    try
    {
      mscl::GeometricVector biasVector = msclInertialNode->captureGyroBias(30000);

      ROS_INFO("Gyro Bias Captured:\nbias_vector[0] = %f\nbias_vector[1] = %f\nbias_vector[2] = %f\n\n",
               biasVector.x(), biasVector.y(), biasVector.z());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Set soft iron matrix values
bool Microstrain::set_soft_iron_matrix(ros_mscl::SetSoftIronMatrix::Request &req,
                                       ros_mscl::SetSoftIronMatrix::Response &res)
{
  res.success = false;
  ROS_INFO("Setting the soft iron matrix values\n");

  if (msclInertialNode)
  {
    try
    {
      mscl::Matrix_3x3 data;
      data.set(0, 0, req.soft_iron_1.x);
      data.set(0, 1, req.soft_iron_1.y);
      data.set(0, 2, req.soft_iron_1.z);
      data.set(1, 0, req.soft_iron_2.x);
      data.set(1, 1, req.soft_iron_2.y);
      data.set(1, 2, req.soft_iron_2.z);
      data.set(2, 0, req.soft_iron_3.x);
      data.set(2, 1, req.soft_iron_3.y);
      data.set(2, 2, req.soft_iron_3.z);

      msclInertialNode->setMagnetometerSoftIronMatrix(data);
      ROS_INFO("Sent values:     [%f  %f  %f][%f  %f  %f][%f  %f  %f]\n",
               data(0, 0), data(0, 1), data(0, 2),
               data(1, 0), data(1, 1), data(1, 2),
               data(2, 0), data(2, 1), data(2, 2));

      data = msclInertialNode->getMagnetometerSoftIronMatrix();

      ROS_INFO("Returned values:     [%f  %f  %f][%f  %f  %f][%f  %f  %f]\n",
               data(0, 0), data(0, 1), data(0, 2),
               data(1, 0), data(1, 1), data(1, 2),
               data(2, 0), data(2, 1), data(2, 2));
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }
  return res.success;
}

// Get soft iron matrix values
bool Microstrain::get_soft_iron_matrix(std_srvs::Trigger::Request &req,
                                       std_srvs::Trigger::Response &res)
{
  res.success = false;
  ROS_INFO("Getting the soft iron matrix values\n");

  if (msclInertialNode)
  {
    try
    {
      mscl::Matrix_3x3 data = msclInertialNode->getMagnetometerSoftIronMatrix();

      ROS_INFO("Soft iron matrix values: [%f  %f  %f][%f  %f  %f][%f  %f  %f]\n",
               data(0, 0), data(0, 1), data(0, 2),
               data(1, 0), data(1, 1), data(1, 2),
               data(2, 0), data(2, 1), data(2, 2));
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Set complementary filter values
bool Microstrain::set_complementary_filter(ros_mscl::SetComplementaryFilter::Request &req,
                                           ros_mscl::SetComplementaryFilter::Response &res)
{
  ROS_INFO("Setting the complementary filter values\n");
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      mscl::ComplementaryFilterData comp_filter_command;
      comp_filter_command.upCompensationEnabled = req.up_comp_enable;
      comp_filter_command.upCompensationTimeInSeconds = req.up_comp_time_const;
      comp_filter_command.northCompensationEnabled = req.north_comp_enable;
      comp_filter_command.northCompensationTimeInSeconds = req.north_comp_time_const;

      msclInertialNode->setComplementaryFilterSettings(comp_filter_command);

      ROS_INFO("Sent values:     Up Enable: %d North Enable: %d Up Time Constant: %f North Time Constant: %f \n",
               comp_filter_command.upCompensationEnabled, comp_filter_command.northCompensationEnabled,
               comp_filter_command.upCompensationTimeInSeconds, comp_filter_command.northCompensationTimeInSeconds);

      comp_filter_command = msclInertialNode->getComplementaryFilterSettings();

      ROS_INFO("Returned values: Up Enable: %d North Enable: %d Up Time Constant: %f North Time Constant: %f \n",
               comp_filter_command.upCompensationEnabled, comp_filter_command.northCompensationEnabled,
               comp_filter_command.upCompensationTimeInSeconds, comp_filter_command.northCompensationTimeInSeconds);
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get complementary filter values
bool Microstrain::get_complementary_filter(std_srvs::Trigger::Request &req,
                                           std_srvs::Trigger::Response &res)
{
  res.success = false;
  ROS_INFO("Getting the complementary filter values\n");
  if (msclInertialNode)
  {
    try
    {
      mscl::ComplementaryFilterData comp_filter_command = msclInertialNode->getComplementaryFilterSettings();

      ROS_INFO("Returned values: Up Enable: %d North Enable: %d Up Time Constant: %f North Time Constant: %f \n",
               comp_filter_command.upCompensationEnabled, comp_filter_command.northCompensationEnabled,
               comp_filter_command.upCompensationTimeInSeconds, comp_filter_command.northCompensationTimeInSeconds);
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Initialize filter with Euler angles
bool Microstrain::set_filter_euler(ros_mscl::SetFilterEuler::Request &req,
                                   ros_mscl::SetFilterEuler::Response &res)
{
  res.success = false;
  ROS_INFO("Initializing the Filter with Euler angles\n");
  if (msclInertialNode)
  {
    try
    {
      mscl::EulerAngles attitude(req.angle.x,
                                 req.angle.y,
                                 req.angle.z);

      msclInertialNode->setInitialAttitude(attitude);
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Set filter with heading angle
bool Microstrain::set_filter_heading(ros_mscl::SetFilterHeading::Request &req,
                                     ros_mscl::SetFilterHeading::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Resetting the Filter\n");
      msclInertialNode->resetFilter();

      ROS_INFO("Initializing the Filter with a heading angle\n");
      msclInertialNode->setInitialHeading(req.angle);
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Set sensor to vehicle frame transformation
bool Microstrain::set_sensor_vehicle_frame_trans(ros_mscl::SetSensorVehicleFrameTrans::Request &req,
                                                 ros_mscl::SetSensorVehicleFrameTrans::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Setting the sensor to vehicle frame transformation\n");
      mscl::EulerAngles angles(req.angle.x, req.angle.y, req.angle.z);
      msclInertialNode->setSensorToVehicleTransformation(angles);

      angles = msclInertialNode->getSensorToVehicleTransformation();
      ROS_INFO("Transformation successfully set.\n");
      ROS_INFO("New angles: %f roll %f pitch %f yaw\n",
               angles.roll(), angles.pitch(), angles.yaw());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get sensor to vehicle frame transformation
bool Microstrain::get_sensor_vehicle_frame_trans(std_srvs::Trigger::Request &req,
                                                 std_srvs::Trigger::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      mscl::EulerAngles angles = msclInertialNode->getSensorToVehicleTransformation();
      ROS_INFO("Sensor Vehicle Frame Transformation Angles: %f roll %f pitch %f yaw\n",
               angles.roll(), angles.pitch(), angles.yaw());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Set reference position
bool Microstrain::set_reference_position(ros_mscl::SetReferencePosition::Request &req,
                                         ros_mscl::SetReferencePosition::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Setting reference Position\n");
      mscl::Position referencePosition(req.position.x, req.position.y, req.position.z);
      mscl::FixedReferencePositionData referencePositionData(true, referencePosition);
      msclInertialNode->setFixedReferencePosition(referencePositionData);
      ROS_INFO("Reference position successfully set\n");
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get reference position
bool Microstrain::get_reference_position(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Getting reference position");
      mscl::Position referencePosition = msclInertialNode->getFixedReferencePosition().referencePosition;
      ROS_INFO("Reference position: Lat %f , Long %f, Alt %f", referencePosition.latitude(),
               referencePosition.longitude(), referencePosition.altitude());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Enable or disable coning and sculling compensation
bool Microstrain::set_coning_sculling_comp(ros_mscl::SetConingScullingComp::Request &req,
                                           ros_mscl::SetConingScullingComp::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("%s Coning and Sculling compensation", req.enable ? "DISABLED" : "ENABLED\n");
      msclInertialNode->setConingAndScullingEnable(req.enable);

      ROS_INFO("Reading Coning and Sculling compensation enabled state:\n");
      bool enabled = msclInertialNode->getConingAndScullingEnable();
      ROS_INFO("%s Coning and Sculling compensation", enabled ? "DISABLED" : "ENABLED\n");
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get coning and sculling compenastion enabled/disabled state
bool Microstrain::get_coning_sculling_comp(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Reading Coning and Sculling compensation enabled state:\n");
      bool enabled = msclInertialNode->getConingAndScullingEnable();
      ROS_INFO("%s Coning and Sculling compensation", enabled ? "DISABLED" : "ENABLED\n");
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Set estimation control filter flags
bool Microstrain::set_estimation_control_flags(ros_mscl::SetEstimationControlFlags::Request &req,
                                               ros_mscl::SetEstimationControlFlags::Response &res)
{
  res.success = true;
  if (msclInertialNode)
  {
    try
    {
      mscl::EstimationControlOptions flags(req.flag);
      msclInertialNode->setEstimationControlFlags(flags);
      flags = msclInertialNode->getEstimationControlFlags();
      ROS_INFO("Estimation control set to: %d", flags.AsUint16());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get estimatio control filter flags
bool Microstrain::get_estimation_control_flags(std_srvs::Trigger::Request &req,
                                               std_srvs::Trigger::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Estimation control set to: %d", msclInertialNode->getEstimationControlFlags().AsUint16());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get device basic status. Variables in basic status struct change based on device model
bool Microstrain::get_basic_status(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  if (!msclInertialNode)
  {
    return false;
  }
  
  if(msclInertialNode->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    mscl::DeviceStatusMap status;
    if(msclInertialNode->features().supportedStatusSelectors().size() > 0)
    {
      mscl::DeviceStatusData statusData = msclInertialNode->getBasicDeviceStatus();
      status = statusData.asMap();
    }
    else
    {
      ROS_INFO("Model Number: \t\t\t\t\t%04u\n", msclInertialNode->modelNumber().c_str());
      return true;
    }
    
    mscl::DeviceStatusMap::iterator it;
    for ( it = status.begin(); it != status.end(); it++ )
    {
      switch (it->first)
      {
      case mscl::DeviceStatusValues::ModelNumber:
        ROS_INFO("Model Number: \t\t\t\t\t%s\n", (it->second).c_str());
        break;
      case mscl::DeviceStatusValues::StatusStructure_Value:
        ROS_INFO("Status Selector: \t\t\t\t%s\n", (it->second).c_str());
        break;
      case mscl::DeviceStatusValues::SystemState_Value:
        ROS_INFO("System state: \t\t\t\t\t%s\n", (it->second).c_str());
        break;
      default:
        break;
      }
    }
  }
  return true;
}

// Get diagnostic status of device. Changes based on device model.
bool Microstrain::get_diagnostic_report(std_srvs::Trigger::Request &req,
                                        std_srvs::Trigger::Response &res)
{
  res.success = false;

  if (!msclInertialNode)
  {    
    return false;
  }
  
  if(msclInertialNode->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    mscl::DeviceStatusMap status;
    if(msclInertialNode->features().supportedStatusSelectors().size() > 1) 
    {
      mscl::DeviceStatusData statusData = msclInertialNode->getDiagnosticDeviceStatus();
      status = statusData.asMap();
      res.success = true;
    }
      
    else if(msclInertialNode->features().supportedStatusSelectors().size() > 0)
    {
      mscl::DeviceStatusData statusData = msclInertialNode->getBasicDeviceStatus();
      status = statusData.asMap();
      res.success = true;
    }
    else
    {
      ROS_INFO("Model Number: \t\t\t\t\t%04u\n", msclInertialNode->modelNumber().c_str());
      return true;
    }
    
    mscl::DeviceStatusMap::iterator it;
    for ( it = status.begin(); it != status.end(); it++ )
    {
      switch (it->first)
      {
      case mscl::DeviceStatusValues::ModelNumber:
        ROS_INFO("Model Number: \t\t\t\t\t%s\n", (it->second).c_str());
        break;
      case mscl::DeviceStatusValues::StatusStructure_Value:
        ROS_INFO("Status Selector: \t\t\t\t%s\n", (it->second).c_str());
        break;
      case mscl::DeviceStatusValues::SystemState_Value:
        ROS_INFO("System state: \t\t\t\t\t%s\n", (it->second).c_str());
        break;
      case mscl::DeviceStatusValues::ImuStreamInfo_Enabled:
        ROS_INFO("IMU Streaming Enabled: \t\t\t\t%s\n", strcmp((it->second).c_str(),"1") == 0 ? "TRUE" : "FALSE");
        break;
      case mscl::DeviceStatusValues::ImuStreamInfo_PacketsDropped:
        ROS_INFO("Number of Dropped IMU Packets: \t\t\t%s Packets\n", (it->second).c_str());
        break;
      case mscl::DeviceStatusValues::EstimationFilterStreamInfo_Enabled:
        ROS_INFO("FILTER Streaming Enabled: \t\t\t%s\n", strcmp((it->second).c_str(),"1") == 0 ? "TRUE" : "FALSE");
        break;
      case mscl::DeviceStatusValues::EstimationFilterStreamInfo_PacketsDropped:
        ROS_INFO("Number of Dropped FILTER Packets: \t\t%s Packets\n", (it->second).c_str());
        break;
      case mscl::DeviceStatusValues::ComPortInfo_BytesWritten:
        ROS_INFO("Communications Port Bytes Written: \t\t%s Bytes\n", (it->second).c_str());
        break;
      case mscl::DeviceStatusValues::ComPortInfo_BytesRead:
        ROS_INFO("Communications Port Bytes Read: \t\t%s Bytes\n", (it->second).c_str());
        break;
      case mscl::DeviceStatusValues::ComPortInfo_OverrunsOnWrite:
        ROS_INFO("Communications Port Write Overruns: \t\t%s\n", (it->second).c_str());
        break;
      case mscl::DeviceStatusValues::ComPortInfo_OverrunsOnRead:
        ROS_INFO("Communications Port Read Overruns: \t\t%s\n", (it->second).c_str());
        break;
      case mscl::DeviceStatusValues::ImuMessageInfo_MessageParsingErrors:
        ROS_INFO("IMU Parser Errors: \t\t\t\t%s Errors\n", (it->second).c_str());
        break;
      case mscl::DeviceStatusValues::ImuMessageInfo_MessagesRead:
        ROS_INFO("IMU Message Count: \t\t\t\t%s Messages\n", (it->second).c_str());
        break;
      case mscl::DeviceStatusValues::ImuMessageInfo_LastMessageReadinMS:
        ROS_INFO("IMU Last Message Received: \t\t\t%s ms\n", (it->second).c_str());
        break;
      default:
        break;
      }
    }
  }
  return res.success;
}

// Set zero angular-rate update threshold
bool Microstrain::set_zero_angle_update_threshold(ros_mscl::SetZeroAngleUpdateThreshold::Request &req,
                                                  ros_mscl::SetZeroAngleUpdateThreshold::Response &res)
{
  res.success = false;
  ROS_INFO("Setting Zero Angular-Rate-Update threshold\n");

  if (msclInertialNode)
  {
    try
    {
      mscl::ZUPTSettingsData ZUPTSettings(req.enable, req.threshold);
      msclInertialNode->setAngularRateZUPT(ZUPTSettings);

      ZUPTSettings = msclInertialNode->getAngularRateZUPT();
      ROS_INFO("Enable value set to: %d, Threshold is: %f rad/s",
               ZUPTSettings.enabled, ZUPTSettings.threshold);
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get zero angular rate update threshold value
bool Microstrain::get_zero_angle_update_threshold(std_srvs::Trigger::Request &req,
                                                  std_srvs::Trigger::Response &res)
{
  res.success = false;
  ROS_INFO("Getting Zero Angular-Rate-Update threshold\n");

  if (msclInertialNode)
  {
    try
    {
      mscl::ZUPTSettingsData ZUPTSettings = msclInertialNode->getAngularRateZUPT();
      ROS_INFO("Enable value set to: %d, Threshold is: %f rad/s",
               ZUPTSettings.enabled, ZUPTSettings.threshold);
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Set tare orientation angle values
bool Microstrain::set_tare_orientation(ros_mscl::SetTareOrientation::Request &req,
                                       ros_mscl::SetTareOrientation::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      mscl::TareAxisValues axisValue(req.axis & mscl::InertialTypes::TARE_PITCH_AXIS,
                                     req.axis & mscl::InertialTypes::TARE_ROLL_AXIS,
                                     req.axis & mscl::InertialTypes::TARE_YAW_AXIS);
      msclInertialNode->tareOrientation(axisValue);
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }
  return res.success;
}

// Set accel noise values
bool Microstrain::set_accel_noise(ros_mscl::SetAccelNoise::Request &req,
                                  ros_mscl::SetAccelNoise::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Setting the accel noise values\n");
      mscl::GeometricVector noise(req.noise.x, req.noise.y, req.noise.z);
      msclInertialNode->setAccelNoiseStandardDeviation(noise);

      noise = msclInertialNode->getAccelNoiseStandardDeviation();
      ROS_INFO("Accel noise values successfully set.\n");
      ROS_INFO("Returned values: %f X %f Y %f Z\n",
               noise.x(), noise.y(), noise.z());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get accel noise values
bool Microstrain::get_accel_noise(std_srvs::Trigger::Request &req,
                                  std_srvs::Trigger::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Getting the accel noise values\n");
      mscl::GeometricVector noise = msclInertialNode->getAccelNoiseStandardDeviation();
      ROS_INFO("Returned values: %f X %f Y %f Z\n",
               noise.x(), noise.y(), noise.z());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Set gyro noise values
bool Microstrain::set_gyro_noise(ros_mscl::SetGyroNoise::Request &req,
                                 ros_mscl::SetGyroNoise::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Setting the gyro noise values\n");
      mscl::GeometricVector noise(req.noise.x, req.noise.y, req.noise.z);
      msclInertialNode->setGyroNoiseStandardDeviation(noise);

      noise = msclInertialNode->getGyroNoiseStandardDeviation();
      ROS_INFO("Gyro noise values successfully set.\n");
      ROS_INFO("Returned values: %f X %f Y %f Z\n",
               noise.x(), noise.y(), noise.z());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get gyro noise values
bool Microstrain::get_gyro_noise(std_srvs::Trigger::Request &req,
                                 std_srvs::Trigger::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Getting the gyro noise values\n");
      mscl::GeometricVector noise = msclInertialNode->getGyroNoiseStandardDeviation();
      ROS_INFO("Gyro noise values: %f X %f Y %f Z\n",
               noise.x(), noise.y(), noise.z());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Set magnetometer noise values
bool Microstrain::set_mag_noise(ros_mscl::SetMagNoise::Request &req,
                                ros_mscl::SetMagNoise::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Setting the mag noise values\n");
      mscl::GeometricVector noise(req.noise.x, req.noise.y, req.noise.z);
      msclInertialNode->setHardIronOffsetProcessNoise(noise);

      noise = msclInertialNode->getHardIronOffsetProcessNoise();
      ROS_INFO("Mag noise values successfully set.\n");
      ROS_INFO("Returned values: %f X %f Y %f Z\n",
               noise.x(), noise.y(), noise.z());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get magnetometer noise values
bool Microstrain::get_mag_noise(std_srvs::Trigger::Request &req,
                                std_srvs::Trigger::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Getting the mag noise values\n");
      mscl::GeometricVector noise = msclInertialNode->getHardIronOffsetProcessNoise();
      ROS_INFO("Returned values: %f X %f Y %f Z\n",
               noise.x(), noise.y(), noise.z());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Set gyro bias model
bool Microstrain::set_gyro_bias_model(ros_mscl::SetGyroBiasModel::Request &req,
                                      ros_mscl::SetGyroBiasModel::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Setting the gyro bias model values\n");
      mscl::GeometricVectors collection;
      mscl::GeometricVector noise(req.noise_vector.x, req.noise_vector.y, req.noise_vector.z);
      collection.push_back(noise);
      mscl::GeometricVector beta_vector(req.beta_vector.x, req.beta_vector.y, req.beta_vector.z);
      collection.push_back(beta_vector);
      msclInertialNode->setGyroBiasModelParams(collection);

      collection = msclInertialNode->getGyroBiasModelParams();
      ROS_INFO("Gyro bias model values successfully set.\n");
      ROS_INFO("Returned values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n",
               collection[0].x(), collection[0].y(), collection[0].z(),
               collection[1].x(), collection[1].y(), collection[1].z());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get gyro bias model
bool Microstrain::get_gyro_bias_model(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Getting the gyro bias model values\n");
      mscl::GeometricVectors collection = msclInertialNode->getGyroBiasModelParams();
      ROS_INFO("Gyro bias model values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n",
               collection[0].x(), collection[0].y(), collection[0].z(),
               collection[1].x(), collection[1].y(), collection[1].z());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }
  return res.success;
}

// Get acces bias model
bool Microstrain::get_accel_bias_model(std_srvs::Trigger::Request &req,
                                       std_srvs::Trigger::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Getting the accel bias model values\n");
      mscl::GeometricVectors collection = msclInertialNode->getAccelBiasModelParams();
      ROS_INFO("Accel bias model values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n",
               collection[0].x(), collection[0].y(), collection[0].z(),
               collection[1].x(), collection[1].y(), collection[1].z());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Set accel bias model
bool Microstrain::set_accel_bias_model(ros_mscl::SetAccelBiasModel::Request &req,
                                       ros_mscl::SetAccelBiasModel::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Setting the accel bias model values\n");
      mscl::GeometricVectors collection;
      mscl::GeometricVector noise(req.noise_vector.x, req.noise_vector.y, req.noise_vector.z);
      collection.push_back(noise);
      mscl::GeometricVector beta_vector(req.beta_vector.x, req.beta_vector.y, req.beta_vector.z);
      collection.push_back(beta_vector);
      msclInertialNode->setAccelBiasModelParams(collection);

      collection = msclInertialNode->getAccelBiasModelParams();
      ROS_INFO("Accel bias model values successfully set.\n");
      ROS_INFO("Returned values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n",
               collection[0].x(), collection[0].y(), collection[0].z(),
               collection[1].x(), collection[1].y(), collection[1].z());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Set accel magnitude error adaptive measurement values
bool Microstrain::set_accel_adaptive_vals(ros_mscl::SetAccelAdaptiveVals::Request &req,
                                          ros_mscl::SetAccelAdaptiveVals::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Setting the accel magnitude error adaptive measurement values\n");
      mscl::AdaptiveMeasurementData adaptiveData;
      adaptiveData.mode = static_cast<mscl::InertialTypes::AdaptiveMeasurementMode>(req.enable);
      adaptiveData.lowPassFilterCutoff = req.low_pass_cutoff;
      adaptiveData.lowLimit = req.low_limit;
      adaptiveData.highLimit = req.high_limit;
      adaptiveData.lowLimitUncertainty = req.low_limit_1sigma;
      adaptiveData.highLimitUncertainty = req.high_limit_1sigma;
      adaptiveData.minUncertainty = req.min_1sigma;

      msclInertialNode->setGravityErrorAdaptiveMeasurement(adaptiveData);

      adaptiveData = msclInertialNode->getGravityErrorAdaptiveMeasurement();
      ROS_INFO("accel magnitude error adaptive measurement values successfully set.\n");
      ROS_INFO("Returned values: Enable: %i, Parameters: %f %f %f %f %f %f",
               adaptiveData.mode, adaptiveData.lowPassFilterCutoff,
               adaptiveData.minUncertainty, adaptiveData.lowLimit,
               adaptiveData.highLimit, adaptiveData.lowLimitUncertainty,
               adaptiveData.highLimitUncertainty);
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get accep magnitude error adaptive measurement values
bool Microstrain::get_accel_adaptive_vals(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Getting the accel magnitude error adaptive measurement values\n");
      mscl::AdaptiveMeasurementData adaptiveData = msclInertialNode->getGravityErrorAdaptiveMeasurement();
      ROS_INFO("Accel magnitude error adaptive measurement values are: Enable: %i, Parameters: %f %f %f %f %f %f",
               adaptiveData.mode, adaptiveData.lowPassFilterCutoff,
               adaptiveData.minUncertainty, adaptiveData.lowLimit,
               adaptiveData.highLimit, adaptiveData.lowLimitUncertainty,
               adaptiveData.highLimitUncertainty);
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Set magnetometer magnitude error adaptive measurement values
bool Microstrain::set_mag_adaptive_vals(ros_mscl::SetMagAdaptiveVals::Request &req,
                                        ros_mscl::SetMagAdaptiveVals::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Setting the mag magnitude error adaptive measurement values\n");
      mscl::AdaptiveMeasurementData adaptiveData;
      adaptiveData.mode = static_cast<mscl::InertialTypes::AdaptiveMeasurementMode>(req.enable);
      adaptiveData.lowPassFilterCutoff = req.low_pass_cutoff;
      adaptiveData.lowLimit = req.low_limit;
      adaptiveData.highLimit = req.high_limit;
      adaptiveData.lowLimitUncertainty = req.low_limit_1sigma;
      adaptiveData.highLimitUncertainty = req.high_limit_1sigma;
      adaptiveData.minUncertainty = req.min_1sigma;

      msclInertialNode->setMagnetometerErrorAdaptiveMeasurement(adaptiveData);

      adaptiveData = msclInertialNode->getMagnetometerErrorAdaptiveMeasurement();
      ROS_INFO("mag magnitude error adaptive measurement values successfully set.\n");
      ROS_INFO("Returned values: Enable: %i, Parameters: %f %f %f %f %f %f",
               adaptiveData.mode, adaptiveData.lowPassFilterCutoff,
               adaptiveData.minUncertainty, adaptiveData.lowLimit,
               adaptiveData.highLimit, adaptiveData.lowLimitUncertainty,
               adaptiveData.highLimitUncertainty);
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get magnetometer magnitude error adaptive measurement values
bool Microstrain::get_mag_adaptive_vals(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Getting the mag magnitude error adaptive measurement values\n");
      mscl::AdaptiveMeasurementData adaptiveData = msclInertialNode->getMagnetometerErrorAdaptiveMeasurement();
      ROS_INFO("Mag magnitude error adaptive measurement values are: Enable: %i, Parameters: %f %f %f %f %f %f",
               adaptiveData.mode, adaptiveData.lowPassFilterCutoff,
               adaptiveData.minUncertainty, adaptiveData.lowLimit,
               adaptiveData.highLimit, adaptiveData.lowLimitUncertainty,
               adaptiveData.highLimitUncertainty);
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get magnetometer dip angle error adaptive measurement values
bool Microstrain::get_mag_dip_adaptive_vals(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Getting the mag dip angle error adaptive measurement values\n");
      mscl::AdaptiveMeasurementData adaptiveData = msclInertialNode->getMagDipAngleErrorAdaptiveMeasurement();
      ROS_INFO("Returned values: Enable: %i, Parameters: %f %f %f %f %f %f",
               adaptiveData.mode, adaptiveData.lowPassFilterCutoff,
               adaptiveData.minUncertainty, adaptiveData.lowLimit,
               adaptiveData.highLimit, adaptiveData.lowLimitUncertainty,
               adaptiveData.highLimitUncertainty);
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get magnetometer dip angle error adaptive measurement values
bool Microstrain::set_mag_dip_adaptive_vals(ros_mscl::SetMagDipAdaptiveVals::Request &req,
                                            ros_mscl::SetMagDipAdaptiveVals::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Setting the mag dip angle error adaptive measurement values\n");
      mscl::AdaptiveMeasurementData adaptiveData;
      adaptiveData.mode = static_cast<mscl::InertialTypes::AdaptiveMeasurementMode>(req.enable);
      adaptiveData.lowPassFilterCutoff = req.low_pass_cutoff;
      adaptiveData.highLimit = req.high_limit;
      adaptiveData.highLimitUncertainty = req.high_limit_1sigma;
      adaptiveData.minUncertainty = req.min_1sigma;

      msclInertialNode->setMagDipAngleErrorAdaptiveMeasurement(adaptiveData);

      adaptiveData = msclInertialNode->getMagDipAngleErrorAdaptiveMeasurement();
      ROS_INFO("mag dip angle error adaptive measurement values successfully set.\n");
      ROS_INFO("Returned values: Enable: %i, Parameters: %f %f %f %f\n",
               adaptiveData.mode, adaptiveData.lowPassFilterCutoff,
               adaptiveData.minUncertainty, adaptiveData.lowLimit,
               adaptiveData.highLimit, adaptiveData.lowLimitUncertainty,
               adaptiveData.highLimitUncertainty);
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get vehicle dynamics mode
bool Microstrain::get_dynamics_mode(std_srvs::Trigger::Request &req,
                                    std_srvs::Trigger::Response &res)
{
  res.success = false;
  ROS_INFO("Getting the vehicle dynamics mode\n");

  if (msclInertialNode)
  {
    try
    {
      mscl::InertialTypes::VehicleModeType mode = msclInertialNode->getVehicleDynamicsMode();
      ROS_INFO("Vehicle dynamics mode is: %d\n", mode);

      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Set vehicle dynamics mode. Only in 45 model.
bool Microstrain::set_dynamics_mode(ros_mscl::SetDynamicsMode::Request &req,
                                    ros_mscl::SetDynamicsMode::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Setting the vehicle dynamics mode\n");
      mscl::InertialTypes::VehicleModeType mode = static_cast<mscl::InertialTypes::VehicleModeType>(req.mode);
      msclInertialNode->setVehicleDynamicsMode(mode);

      mode = msclInertialNode->getVehicleDynamicsMode();
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Set sensor to vehicle frame offset. Only in 45
bool Microstrain::set_sensor_vehicle_frame_offset(ros_mscl::SetSensorVehicleFrameOffset::Request &req,
                                                  ros_mscl::SetSensorVehicleFrameOffset::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Setting the sensor to vehicle frame offset\n");
      mscl::PositionOffset offset(req.offset.x, req.offset.y, req.offset.z);
      msclInertialNode->setSensorToVehicleOffset(offset);

      offset = msclInertialNode->getSensorToVehicleOffset();
      ROS_INFO("Offset successfully set.\n");
      ROS_INFO("Returned offset: %f X %f Y %f Z\n",
               offset.x(), offset.y(), offset.z());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

// Get sensor to vehicle frame offset. Only in 45 model.
bool Microstrain::get_sensor_vehicle_frame_offset(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = false;
  if (msclInertialNode)
  {
    try
    {
      ROS_INFO("Getting the sensor to vehicle frame offset\n");
      mscl::PositionOffset offset = msclInertialNode->getSensorToVehicleOffset();
      ROS_INFO("Returned offset: %f X %f Y %f Z\n",
               offset.x(), offset.y(), offset.z());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

void Microstrain::parseMipPacket(const mscl::MipDataPacket &packet)
{
  switch (packet.descriptorSet())
  {
  case mscl::MipTypes::DataClass::CLASS_AHRS_IMU:
    parseSensorPacket(packet);
    print_packet_stats();
    break;

  case mscl::MipTypes::DataClass::CLASS_ESTFILTER:
    parseEstFilterPacket(packet);
    print_packet_stats();
    break;

  case mscl::MipTypes::DataClass::CLASS_GNSS:
    parseGnssPacket(packet);
    print_packet_stats();
    break;

  default:
    break;
  }
}

void Microstrain::parseSensorPacket(const mscl::MipDataPacket &packet)
{
  ahrs_valid_packet_count_++;
  
  uint64_t time = packet.collectedTimestamp().nanoseconds();
  
  if (packet.hasDeviceTime() && packet.deviceTimeValid()) 
  {
     time = packet.deviceTimestamp().nanoseconds();
  }

  const mscl::MipDataPoints &points = packet.data();

  imu_msg_.header.seq = ahrs_valid_packet_count_;
  imu_msg_.header.stamp = ros::Time().fromNSec ( time );
  imu_msg_.header.frame_id = imu_frame_id_;

  bool hasScaledAccel = false;
  bool hasScaledGyro = false;
  bool hasOrientationQuat = false;

  for (mscl::MipDataPoint point : points)
  {
    switch (point.field())
    {
    //Scaled Accel
    case mscl::MipTypes::CH_FIELD_SENSOR_SCALED_ACCEL_VEC:
    {
      hasScaledAccel = true;

      // Stuff into ROS message - acceleration in m/s^2
      if (point.qualifier() == mscl::MipTypes::CH_X)
      {
        imu_msg_.linear_acceleration.x = 9.81 * point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_Y)
      {
        imu_msg_.linear_acceleration.y = 9.81 * point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_Z)
      {
        imu_msg_.linear_acceleration.z = 9.81 * point.as_float();
      }
    }
    break;

    //Scaled Gyro
    case mscl::MipTypes::CH_FIELD_SENSOR_SCALED_GYRO_VEC:
    {
      hasScaledGyro = true;

      if (point.qualifier() == mscl::MipTypes::CH_X)
      {
        imu_msg_.angular_velocity.x = point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_Y)
      {
        imu_msg_.angular_velocity.y = point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_Z)
      {
        imu_msg_.angular_velocity.z = point.as_float();
      }
    }
    break;

    //Scaled Mag
    case mscl::MipTypes::CH_FIELD_SENSOR_SCALED_MAG_VEC:
    {
      if (point.qualifier() == mscl::MipTypes::CH_X)
      {
        curr_ahrs_mag_x = point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_Y)
      {
        curr_ahrs_mag_y = point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_Z)
      {
        curr_ahrs_mag_z = point.as_float();
      }
    }
    break;

    //Orientation Quaternion
    case mscl::MipTypes::CH_FIELD_SENSOR_ORIENTATION_QUATERNION:
    {
      hasOrientationQuat = true;

      if (point.qualifier() == mscl::MipTypes::CH_QUATERNION)
      {
        mscl::Vector quaternion = point.as_Vector();
        curr_filter_quaternion_ = quaternion;

        // put into ENU - swap X/Y, invert Z
        imu_msg_.orientation.x = quaternion.as_floatAt(2);
        imu_msg_.orientation.y = quaternion.as_floatAt(1);
        imu_msg_.orientation.z = -1.0 * quaternion.as_floatAt(3);
        imu_msg_.orientation.w = quaternion.as_floatAt(0);
      }
    }
    break;
    }
  }

  if (hasScaledAccel)
  {
    // Since the sensor does not produce a covariance for linear acceleration, set it based on our pulled in parameters.
    std::copy(imu_linear_cov_.begin(), imu_linear_cov_.end(), imu_msg_.linear_acceleration_covariance.begin());
  }

  if (hasScaledGyro)
  {
    // Since the sensor does not produce a covariance for angular velocity, set it based on our pulled in parameters.
    std::copy(imu_angular_cov_.begin(), imu_angular_cov_.end(), imu_msg_.angular_velocity_covariance.begin());
  }

  if (hasOrientationQuat)
  {
    // Since the MIP_AHRS data does not contain uncertainty values we have to set them based on the parameter values.
    std::copy(imu_orientation_cov_.begin(), imu_orientation_cov_.end(), imu_msg_.orientation_covariance.begin());
  }

  // Publish
  imu_pub_.publish(imu_msg_);
}

void Microstrain::parseEstFilterPacket(const mscl::MipDataPacket &packet)
{
  filter_valid_packet_count_++;

  uint64_t time = packet.collectedTimestamp().nanoseconds();
  
  if (packet.hasDeviceTime() && packet.deviceTimeValid()) 
  {
     time = packet.deviceTimestamp().nanoseconds();
  }

  const mscl::MipDataPoints &points = packet.data();
  filtered_imu_msg_.header.seq = filter_valid_packet_count_;
  filtered_imu_msg_.header.stamp = ros::Time().fromNSec ( time );
  filtered_imu_msg_.header.frame_id = odom_frame_id_;
  
  nav_msg_.header.seq = filter_valid_packet_count_;
  nav_msg_.header.stamp = ros::Time().fromNSec ( time );
  nav_msg_.header.frame_id = odom_frame_id_;

  bool hasNedVelocity = false;

  for (mscl::MipDataPoint point : points)
  {
    //ROS_INFO("Parsing Points...");
    switch (point.field())
    {
    //Estimated LLH Position
    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_LLH_POS:
    {
      nav_msg_.child_frame_id = odom_child_frame_id_;

      if (point.qualifier() == mscl::MipTypes::CH_LATITUDE)
      {
        curr_filter_posLat = point.as_float();
        nav_msg_.pose.pose.position.y = curr_filter_posLat;
      }
      else if (point.qualifier() == mscl::MipTypes::CH_LONGITUDE)
      {
        curr_filter_posLong = point.as_float();
        nav_msg_.pose.pose.position.x = curr_filter_posLong;
      }
      else if (point.qualifier() == mscl::MipTypes::CH_HEIGHT_ABOVE_ELLIPSOID)
      {
        curr_filter_posHeight = point.as_float();
        nav_msg_.pose.pose.position.z = curr_filter_posHeight;
      }
      
      filtered_imu_msg_.linear_acceleration.x = curr_filter_posLong;
      filtered_imu_msg_.linear_acceleration.y = curr_filter_posLat;
      filtered_imu_msg_.linear_acceleration.z = curr_filter_posHeight;
    }
    break;

    //Estimated NED Velocity
    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_NED_VELOCITY: 
    {
      if (point.qualifier() == mscl::MipTypes::CH_NORTH)
      {
        curr_filter_velNorth = point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_EAST)
      {
        curr_filter_velEast = point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_DOWN)
      {
        curr_filter_velDown = point.as_float();
      }
      
    }
    break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER:
    {
      if (point.qualifier() == mscl::MipTypes::CH_ROLL)
      {
        curr_filter_roll = point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_PITCH)
      {
        curr_filter_pitch = point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_YAW)
      {
        curr_filter_yaw = point.as_float();
      }
    }
    break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION:
    { 
      mscl::Vector quaternion = point.as_Vector();
      curr_filter_quaternion_ = quaternion;
      
      // put into ENU - swap X/Y, invert Z
      nav_msg_.pose.pose.orientation.x = quaternion.as_floatAt(2);
      nav_msg_.pose.pose.orientation.y = quaternion.as_floatAt(1);
      nav_msg_.pose.pose.orientation.z = quaternion.as_floatAt(3) * -1;
      nav_msg_.pose.pose.orientation.w = quaternion.as_floatAt(0);
      filtered_imu_msg_.orientation = nav_msg_.pose.pose.orientation;
    }
    break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE:
    {
      if (point.qualifier() == mscl::MipTypes::CH_X)
      {
        curr_filter_angularRate_x = point.as_float();
        nav_msg_.twist.twist.angular.x = curr_filter_angularRate_x;
        filtered_imu_msg_.angular_velocity.x = curr_filter_angularRate_x;
      }
      else if (point.qualifier() == mscl::MipTypes::CH_Y)
      {
        curr_filter_angularRate_y = point.as_float();
        nav_msg_.twist.twist.angular.y = curr_filter_angularRate_y;
        filtered_imu_msg_.angular_velocity.y = curr_filter_angularRate_y;
      }
      else if (point.qualifier() == mscl::MipTypes::CH_Z)
      {
        curr_filter_angularRate_z = point.as_float();
        nav_msg_.twist.twist.angular.z = curr_filter_angularRate_z;
        filtered_imu_msg_.angular_velocity.z = curr_filter_angularRate_z;
      }
    }
    break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_LLH_UNCERT:
    {
      if (point.qualifier() == mscl::MipTypes::CH_NORTH)
      {
        curr_filter_pos_uncert_north = point.as_float();
        nav_msg_.pose.covariance[7] = curr_filter_pos_uncert_north * curr_filter_pos_uncert_north;
      }
      else if (point.qualifier() == mscl::MipTypes::CH_EAST)
      {
        curr_filter_pos_uncert_east = point.as_float();
        nav_msg_.pose.covariance[0] = curr_filter_pos_uncert_east * curr_filter_pos_uncert_east;
      }
      else if (point.qualifier() == mscl::MipTypes::CH_DOWN)
      {
        curr_filter_pos_uncert_down = point.as_float();
        nav_msg_.pose.covariance[14] = curr_filter_pos_uncert_down * curr_filter_pos_uncert_down;
      }
    }
    break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ATT_UNCERT_EULER:
    {
      if (point.qualifier() == mscl::MipTypes::CH_ROLL)
      {
        curr_filter_att_uncert_roll = point.as_float();
        nav_msg_.pose.covariance[21] = curr_filter_att_uncert_roll * curr_filter_att_uncert_roll;
        filtered_imu_msg_.orientation_covariance[0] = nav_msg_.pose.covariance[21];
      }
      else if (point.qualifier() == mscl::MipTypes::CH_PITCH)
      {
        curr_filter_att_uncert_pitch = point.as_float();
        nav_msg_.pose.covariance[28] = curr_filter_att_uncert_pitch * curr_filter_att_uncert_pitch;
        filtered_imu_msg_.orientation_covariance[4] = nav_msg_.pose.covariance[28];
      }
      else if (point.qualifier() == mscl::MipTypes::CH_YAW)
      {
        curr_filter_att_uncert_yaw = point.as_float();
        nav_msg_.pose.covariance[35] = curr_filter_att_uncert_yaw * curr_filter_att_uncert_yaw;
        filtered_imu_msg_.orientation_covariance[8] = nav_msg_.pose.covariance[35];
      }
    }
    break;

    default:
      break;
    }
  }

  //TODO: only do these if all the required fields are enabled/collected?
  if (hasNedVelocity)
  {
    // rotate velocities from NED to sensor coordinates
    // Constructor takes x, y, z, w
    tf2::Quaternion nav_quat(curr_filter_quaternion_.as_floatAt(2),
                             curr_filter_quaternion_.as_floatAt(1),
                             curr_filter_quaternion_.as_floatAt(3) * -1.0,
                             curr_filter_quaternion_.as_floatAt(0));

    tf2::Vector3 vel_enu(curr_filter_velEast,
                         curr_filter_velNorth,
                         curr_filter_velDown * -1.0);

    tf2::Vector3 vel_in_sensor_frame = tf2::quatRotate(nav_quat.inverse(), vel_enu);

    nav_msg_.twist.twist.linear.x = vel_in_sensor_frame[0];
    nav_msg_.twist.twist.linear.y = vel_in_sensor_frame[1];
    nav_msg_.twist.twist.linear.z = vel_in_sensor_frame[2];
    
    filtered_imu_msg_.linear_acceleration.x = nav_msg_.twist.twist.linear.x;
    filtered_imu_msg_.linear_acceleration.y = nav_msg_.twist.twist.linear.y;
    filtered_imu_msg_.linear_acceleration.z = nav_msg_.twist.twist.linear.z;
  }
  
  std::copy(imu_linear_cov_.begin(), imu_linear_cov_.end(),
              filtered_imu_msg_.linear_acceleration_covariance.begin());
  std::copy(imu_angular_cov_.begin(), imu_angular_cov_.end(),
              filtered_imu_msg_.angular_velocity_covariance.begin());

  // Publish
  filtered_imu_pub_.publish(filtered_imu_msg_);
  nav_pub_.publish(nav_msg_);
}

void Microstrain::parseGnssPacket(const mscl::MipDataPacket &packet)
{
  gps_valid_packet_count_++;
  
  uint64_t time = packet.collectedTimestamp().nanoseconds();
  
  if (packet.hasDeviceTime() && packet.deviceTimeValid()) 
  {
     time = packet.deviceTimestamp().nanoseconds();
  }

  const mscl::MipDataPoints &points = packet.data();

  gps_msg_.header.seq = gps_valid_packet_count_;
  gps_msg_.header.stamp = ros::Time().fromNSec ( time );
  gps_msg_.header.frame_id = gps_frame_id_;

  for (mscl::MipDataPoint point : points)
  {
    switch (point.field())
    {
    //Scaled Accel
    case mscl::MipTypes::CH_FIELD_GNSS_LLH_POSITION:
    {
      if (point.qualifier() == mscl::MipTypes::CH_LATITUDE)
      {
        gps_msg_.latitude = point.as_double();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_LONGITUDE)
      {
        gps_msg_.longitude = point.as_double();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_HEIGHT_ABOVE_ELLIPSOID)
      {
        gps_msg_.altitude = point.as_double();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_HEIGHT_ABOVE_MSL)
      {
        gps_msg_.position_covariance[0] = point.as_double();
        gps_msg_.position_covariance[0] *= gps_msg_.position_covariance[0];
      }
      else if (point.qualifier() == mscl::MipTypes::CH_HORIZONTAL_ACCURACY)
      {
        gps_msg_.position_covariance[4] = point.as_float();
        gps_msg_.position_covariance[4] *= gps_msg_.position_covariance[4];
      }
      else if (point.qualifier() == mscl::MipTypes::CH_VERTICAL_ACCURACY)
      {
        gps_msg_.position_covariance[8] = point.as_float();
        gps_msg_.position_covariance[8] *= gps_msg_.position_covariance[8];
      }
    }

      //gps_msg_.status.status = curr_llh_pos_.valid_flags - 1;
      gps_msg_.status.service = 1;
      gps_msg_.position_covariance_type = 2;

      break;
    }
  }

  // Publish
  gps_pub_.publish(gps_msg_);
}

// Send diagnostic information to device status topic and diagnostic aggregator
void Microstrain::device_status_callback()
{
  if (!msclInertialNode)
  {    
    return;
  }
  
  if(msclInertialNode->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    if(msclInertialNode->features().supportedStatusSelectors().size() > 1) 
    {
      mscl::DeviceStatusData statusData = msclInertialNode->getDiagnosticDeviceStatus();
      mscl::DeviceStatusMap status = statusData.asMap();
      
      mscl::DeviceStatusMap::iterator it;
      for ( it = status.begin(); it != status.end(); it++ )
      {
        switch (it->first)
        {
        case mscl::DeviceStatusValues::ModelNumber:
          device_status_msg_.device_model = atoi(it->second.c_str());
          break;
        case mscl::DeviceStatusValues::StatusStructure_Value:
          device_status_msg_.status_selector = atoi(it->second.c_str());
          break;
        case mscl::DeviceStatusValues::SystemState_Value:
          device_status_msg_.system_state = atoi(it->second.c_str());
          break;
        case mscl::DeviceStatusValues::ImuStreamInfo_Enabled:
          device_status_msg_.imu_stream_enabled = strcmp(it->second.c_str(),"1");
          break;
        case mscl::DeviceStatusValues::ImuStreamInfo_PacketsDropped:
          device_status_msg_.imu_dropped_packets = atoi(it->second.c_str());
          break;
        case mscl::DeviceStatusValues::EstimationFilterStreamInfo_Enabled:
          device_status_msg_.filter_stream_enabled = strcmp(it->second.c_str(),"1");
          break;
        case mscl::DeviceStatusValues::EstimationFilterStreamInfo_PacketsDropped:
          device_status_msg_.filter_dropped_packets = atoi(it->second.c_str());
          break;
        case mscl::DeviceStatusValues::ComPortInfo_BytesWritten:
          device_status_msg_.com1_port_bytes_written = atoi(it->second.c_str());
          break;
        case mscl::DeviceStatusValues::ComPortInfo_BytesRead:
          device_status_msg_.com1_port_bytes_read = atoi(it->second.c_str());
          break;
        case mscl::DeviceStatusValues::ComPortInfo_OverrunsOnWrite:
          device_status_msg_.com1_port_write_overruns = atoi(it->second.c_str());
          break;
        case mscl::DeviceStatusValues::ComPortInfo_OverrunsOnRead:
          device_status_msg_.com1_port_read_overruns = atoi(it->second.c_str());
          break;
        case mscl::DeviceStatusValues::ImuMessageInfo_MessageParsingErrors:
          device_status_msg_.imu_parser_errors = atoi(it->second.c_str());
          break;
        case mscl::DeviceStatusValues::ImuMessageInfo_MessagesRead:
          device_status_msg_.imu_message_count = atoi(it->second.c_str());
          break;
        case mscl::DeviceStatusValues::ImuMessageInfo_LastMessageReadinMS:
          device_status_msg_.imu_last_message_ms = atoi(it->second.c_str());
          break;
        default:
          break;
        }
      }

      device_status_pub_.publish(device_status_msg_);
    }
  }
}

void Microstrain::print_packet_stats()
{
  if (msclInertialNode)
  {    
    return;
  }
  
  if(msclInertialNode->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    if(msclInertialNode->features().supportedStatusSelectors().size() > 1) 
    {
      mscl::DeviceStatusData status = msclInertialNode->getDiagnosticDeviceStatus();

      ahrs_valid_packet_count_ = status.imuMessageInfo().messagesRead;
      ahrs_checksum_error_packet_count_ = status.imuMessageInfo().messageParsingErrors;
      ahrs_timeout_packet_count_ = status.imuStreamInfo().outgoingPacketsDropped;
      filter_timeout_packet_count_ = status.estimationFilterStreamInfo().outgoingPacketsDropped;
      ROS_DEBUG_THROTTLE(1.0, "%u AHRS (%u errors) Packets",
                           ahrs_valid_packet_count_, ahrs_timeout_packet_count_ + ahrs_checksum_error_packet_count_);
      
      gps_checksum_error_packet_count_ = status.gnssMessageInfo().messageParsingErrors;
      gps_valid_packet_count_ = status.gnssMessageInfo().messagesRead;
      gps_timeout_packet_count_ = status.gnssStreamInfo().outgoingPacketsDropped;

      ROS_DEBUG_THROTTLE(1.0, "%u FILTER (%u errors)    %u AHRS (%u errors)    %u GPS (%u errors) Packets",
                         filter_valid_packet_count_, filter_timeout_packet_count_,
                         ahrs_valid_packet_count_, ahrs_timeout_packet_count_ + ahrs_checksum_error_packet_count_,
                         gps_valid_packet_count_, gps_timeout_packet_count_ + gps_checksum_error_packet_count_);
      ROS_DEBUG_THROTTLE(1.0, "%u FILTER (%u errors)    %u AHRS (%u errors) Packets",
                         filter_valid_packet_count_, filter_timeout_packet_count_,
                         ahrs_valid_packet_count_, ahrs_timeout_packet_count_ + ahrs_checksum_error_packet_count_);
    }
  }
}

} // namespace Microstrain
