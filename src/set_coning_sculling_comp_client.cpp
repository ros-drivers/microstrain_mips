#include "ros/ros.h"
#include "microstrain_3dm_gx5_45/SetConingScullingComp.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "set_coning_sculling_comp_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_3dm_gx5_45::SetConingScullingComp>("SetConingScullingComp");
  microstrain_3dm_gx5_45::SetConingScullingComp srv;

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
