#include "ros/ros.h"
#include "ros_mscl/SetFilterEuler.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "set_filter_euler_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ros_mscl::SetFilterEuler>("SetFilterEuler");
  ros_mscl::SetFilterEuler srv;

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
