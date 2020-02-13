#include "ros/ros.h"
#include "ros_mscl/SetConingScullingComp.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "set_coning_sculling_comp_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ros_mscl::SetConingScullingComp>("SetConingScullingComp");
  ros_mscl::SetConingScullingComp srv;

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
