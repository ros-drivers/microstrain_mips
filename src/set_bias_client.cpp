#include "ros/ros.h"
#include "microstrain_3dm_gx5_45/SetBias.h"
#include <cstdlib>


int main(){

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_3dm_gx5_45::SetBias>("set_bias_values");
  microstrain_3dm_gx5_45::SetBias srv;

  if (client.call(srv))
  {
      if (srv.response.success)
      {
        ROS_INFO("success");
      }
  }
  return 0;
}
