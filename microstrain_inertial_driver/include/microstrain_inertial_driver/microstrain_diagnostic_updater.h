/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord GX5-Series Driver Definition File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c)  2020, Parker Hannifin Corp
//
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef MICROSTRAIN_INERTIAL_DRIVER_MICROSTRAIN_DIAGNOSTIC_UPDATER_H
#define MICROSTRAIN_INERTIAL_DRIVER_MICROSTRAIN_DIAGNOSTIC_UPDATER_H

#include "ros/ros.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"
#include "microstrain_inertial_msgs/Status.h"
#include "microstrain_inertial_driver/microstrain_inertial_driver.h"

#include <string>

namespace ros_mscl
{
class RosDiagnosticUpdater : private diagnostic_updater::Updater
{
public:
  RosDiagnosticUpdater();

  void generalDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void packetDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void portDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void imuDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void statusCallback(const microstrain_inertial_msgs::Status::ConstPtr& status);

private:
  ros::NodeHandle nh_;
  ros::Subscriber status_sub_;

  microstrain_inertial_msgs::Status last_status_;
};
}  // namespace ros_mscl

#endif  // MICROSTRAIN_INERTIAL_DRIVER_MICROSTRAIN_DIAGNOSTIC_UPDATER_H
