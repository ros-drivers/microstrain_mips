#include "ros/ros.h"
#include "microstrain_3dm_gx5/SetZeroAngleUpdateThreshold.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "set_zero_angle_update_threshold_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_3dm_gx5::SetZeroAngleUpdateThreshold>("SetZeroAngleUpdateThreshold");
  microstrain_3dm_gx5::SetZeroAngleUpdateThreshold srv;

  srv.request.enable = atoll(argv[1]);
  srv.request.threshold = atoll(argv[2]);

  if (client.call(srv))
  {
      if (srv.response.success)
      {
        ROS_INFO("success");
      }
  }
  else
  {
    ROS_INFO("Failed to call service");
  }
  return 0;
}
