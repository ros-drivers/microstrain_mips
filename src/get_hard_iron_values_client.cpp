#include "ros/ros.h"
#include "microstrain_3dm/GetHardIronValues.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "get_hard_iron_values");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_3dm::GetHardIronValues>("GetHardIronValues");
  microstrain_3dm::GetHardIronValues srv;

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
