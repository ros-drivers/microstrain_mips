#include "ros/ros.h"
#include "microstrain_3dm_gx5/GetAccelAdaptiveVals.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "get_accel_adaptive_vals_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_3dm_gx5::GetAccelAdaptiveVals>("GetAccelAdaptiveVals");
  microstrain_3dm_gx5::GetAccelAdaptiveVals srv;

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
