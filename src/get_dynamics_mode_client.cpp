#include "ros/ros.h"
#include "microstrain_3dm/GetDynamicsMode.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "get_dynamics_mode");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_3dm::GetDynamicsMode>("GetDynamicsMode");
  microstrain_3dm::GetDynamicsMode srv;

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
