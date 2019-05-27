#include "ros/ros.h"
#include "microstrain_mips/SetConingScullingComp.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "set_coning_sculling_comp_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_mips::SetConingScullingComp>("SetConingScullingComp");
  microstrain_mips::SetConingScullingComp srv;

  srv.request.enable = atoll(argv[1]);

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
