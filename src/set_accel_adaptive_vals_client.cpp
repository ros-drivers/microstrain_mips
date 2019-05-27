#include "ros/ros.h"
#include "microstrain_mips/SetAccelAdaptiveVals.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "set_accel_adaptive_vals_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_mips::SetAccelAdaptiveVals>("SetAccelAdaptiveVals");
  microstrain_mips::SetAccelAdaptiveVals srv;

  srv.request.enable = atoll(argv[1]);
  srv.request.low_pass_cutoff = atoll(argv[2]);
  srv.request.min_1sigma = atoll(argv[3]);
  srv.request.low_limit = atoll(argv[4]);
  srv.request.high_limit = atoll(argv[5]);
  srv.request.low_limit_1sigma = atoll(argv[6]);
  srv.request.high_limit_1sigma = atoll(argv[7]);



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
