#include "ros/ros.h"
#include "ros_mscl/InitFilterHeading.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "init_filter_heading_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ros_mscl::InitFilterHeading>("InitFilterHeading");
  ros_mscl::InitFilterHeading srv;

  srv.request.angle = atoll(argv[1]);

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
