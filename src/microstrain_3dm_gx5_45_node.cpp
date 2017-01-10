/**
 * @file    microstrain_3dm_gx5_45.cpp
 * @author  Brian S. Bingham
 * @version 0.1
 *
 * @brief ROS Node for Microstrain
 *
 */


#include "microstrain_3dm_gx5_45.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "microstrain_3dm_gx5_45_node");
  Microstrain::Microstrain ustrain;
  ustrain.run();
  return 0;
}
