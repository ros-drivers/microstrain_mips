#include "ros/ros.h"
#include "microstrain_3dm_gx5_45/bias_values.h"
#include <cstdlib>

int main(){

  ros::NodeHandle n;
  ros::serviceClient client = n.serviceClient<microstrain_3dm_gx5_45::bias_values>("bias_values");
  microstrain_3dm_gx5_45::bias_values srv;

  if (client.call(srv))
  {
      ROS_INFO(srv.response.bias_data_vector);
  }
  return 0;
}
