#include "ros/ros.h"
#include "microstrain_mips/SetMagDipAdaptiveVals.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "set_mag_dip_adaptive_vals_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_mips::SetMagDipAdaptiveVals>("SetMagDipAdaptiveVals");
  microstrain_mips::SetMagDipAdaptiveVals srv;

  srv.request.enable = atoll(argv[1]);
  srv.request.low_pass_cutoff = atoll(argv[2]);
  srv.request.min_1sigma = atoll(argv[3]);
  srv.request.high_limit = atoll(argv[4]);
  srv.request.high_limit_1sigma = atoll(argv[5]);



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
