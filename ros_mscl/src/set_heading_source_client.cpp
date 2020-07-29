#include "ros/ros.h"
#include "ros_mscl/SetHeadingSource.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "set_heading_source_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ros_mscl::SetHeadingSource>("SetHeadingSource");
  ros_mscl::SetHeadingSource srv;

  srv.request.headingSource = atoll(argv[1]);

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
