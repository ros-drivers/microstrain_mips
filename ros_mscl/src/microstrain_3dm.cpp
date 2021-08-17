/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <algorithm>
#include <time.h>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include <ctime>

#include <ros/callback_queue.h>
#include <tf2/LinearMath/Transform.h>

#include "mscl/mscl.h"
#include "ros_mscl/microstrain_3dm.h"
#include "ros_mscl/microstrain_diagnostic_updater.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization
/////////////////////////////////////////////////////////////////////////////////////////////////////

namespace microstrain
{
/////////////////////////////////////////////////////////////////////////////////////////////////////
// Run Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Microstrain::run()
{
  // ROS setup
  ros::Time::init();
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  try
  {
    // Configure the device and setup publishers/services/subscribers
    if (!MicrostrainNodeBase::initialize(&node))
      ROS_FATAL("Unable to initialize node base");
    if (!MicrostrainNodeBase::configure(&private_nh))
      ROS_FATAL("Unable to configure node base");

    ros::Rate r(static_cast<int64_t>(timer_update_rate_hz_));

    ros_mscl::RosDiagnosticUpdater ros_diagnostic_updater;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    //
    // Main packet processing loop
    //
    ROS_INFO("Starting Data Parsing");
    while (ros::ok())
    {
      // Parse the data from the device and publish it
      parse_and_publish();

      // Take care of service requests
      ros::spinOnce();

      // Be nice
      r.sleep();
    }
  }
  catch (mscl::Error_Connection)
  {
    ROS_ERROR("Device Disconnected");
  }
  catch (mscl::Error& e)
  {
    ROS_FATAL("Error: %s", e.what());
  }

  // Release the inertial node, if necessary
  if (config_.inertial_device_)
  {
    config_.inertial_device_->setToIdle();
    config_.inertial_device_->connection().disconnect();
  }

  // Close raw data file if enabled
  if (config_.raw_file_enable_)
  {
    config_.raw_file_.close();
  }
}

}  // namespace microstrain
