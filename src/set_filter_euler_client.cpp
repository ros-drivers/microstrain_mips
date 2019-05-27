#include "ros/ros.h"
#include "microstrain_mips/SetFilterEuler.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "set_filter_euler_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_mips::SetFilterEuler>("SetFilterEuler");
  microstrain_mips::SetFilterEuler srv;

  srv.request.angle.x = atoll(argv[1]);
  srv.request.angle.y = atoll(argv[2]);
  srv.request.angle.z = atoll(argv[3]);


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
