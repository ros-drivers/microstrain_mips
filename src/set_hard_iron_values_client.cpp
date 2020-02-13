#include "ros/ros.h"
#include "ros_mscl/SetHardIronValues.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "set_hard_iron_values");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ros_mscl::SetHardIronValues>("SetHardIronValues");
  ros_mscl::SetHardIronValues srv;

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
