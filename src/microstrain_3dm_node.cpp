/*

Copyright (c) 2017, Brian Bingham

This code is licensed under MIT license (see LICENSE file for details)

*/

/**
 * @file    microstrain_3dm_gx5_45.cpp
 * @author  Brian S. Bingham
 * @version 0.1
 *
 * @brief ROS Node for Microstrain
 *
 */


#include "microstrain_3dm.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "microstrain_mips_node");
  Microstrain::Microstrain ustrain;
  ustrain.run();
  ros::shutdown();
  return 0;
}
