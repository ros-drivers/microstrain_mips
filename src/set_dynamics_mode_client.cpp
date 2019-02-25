#include "ros/ros.h"
#include "microstrain_3dm_gx5/SetDynamicsMode.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "set_dynamics_mode_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_3dm_gx5::SetDynamicsMode>("SetDynamicsMode");
  microstrain_3dm_gx5::SetDynamicsMode srv;

  srv.request.mode = atoll(argv[1]);

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
