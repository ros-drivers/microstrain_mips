#include "ros/ros.h"
#include "microstrain_3dm_gx5/SetGyroBias.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "set_gyro_bias");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_3dm_gx5::SetGyroBias>("SetGyroBias");
  microstrain_3dm_gx5::SetGyroBias srv;

  srv.request.bias.x = atoll(argv[1]);
  srv.request.bias.y = atoll(argv[2]);
  srv.request.bias.z = atoll(argv[3]);


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
