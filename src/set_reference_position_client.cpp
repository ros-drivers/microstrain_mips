#include "ros/ros.h"
#include "ros_mscl/SetReferencePosition.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "set_reference_position_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ros_mscl::SetReferencePosition>("SetReferencePosition");
  ros_mscl::SetReferencePosition srv;

  srv.request.position.x = atoll(argv[1]);
  srv.request.position.y = atoll(argv[2]);
  srv.request.position.z = atoll(argv[3]);

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
