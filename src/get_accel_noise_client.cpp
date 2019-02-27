#include "ros/ros.h"
#include "microstrain_3dm_gx5/GetAccelNoise.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "get_accel_noise_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_3dm_gx5::GetAccelNoise>("GetAccelNoise");
  microstrain_3dm_gx5::GetAccelNoise srv;

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
