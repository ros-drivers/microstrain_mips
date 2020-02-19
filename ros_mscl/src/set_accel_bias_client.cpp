#include "ros/ros.h"
#include "ros_mscl/SetAccelBias.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "set_accel_bias");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ros_mscl::SetAccelBias>("SetAccelBias");
  ros_mscl::SetAccelBias srv;

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
