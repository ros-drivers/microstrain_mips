#include "ros/ros.h"
#include "microstrain_3dm/GetMagDipAdaptiveVals.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "get_mag_dip_adaptive_vals");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_3dm::GetMagDipAdaptiveVals>("GetMagDipAdaptiveVals");
  microstrain_3dm::GetMagDipAdaptiveVals srv;

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
