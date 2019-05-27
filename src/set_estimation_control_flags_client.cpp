#include "ros/ros.h"
#include "microstrain_mips/SetEstimationControlFlags.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "set_estimation_control_flags_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_mips::SetEstimationControlFlags>("SetEstimationControlFlags");
  microstrain_mips::SetEstimationControlFlags srv;

  srv.request.flag = atoll(argv[1]);

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
