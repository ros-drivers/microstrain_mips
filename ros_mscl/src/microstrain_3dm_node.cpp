/*

Copyright (c) 2017, Brian Bingham
Copyright (c)  2020, Parker Hannifin Corp
This code is licensed under MIT license (see LICENSE file for details)

*/

#include "ros_mscl/microstrain_3dm.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_mscl_node");
  microstrain::Microstrain ustrain;
  ustrain.run();
  ros::shutdown();
  return 0;
}
