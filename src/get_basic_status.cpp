#include "ros/ros.h"
#include "microstrain_3dm/GetBasicStatus.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "gyro_bias_capture_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_3dm::GyroBiasCapture>("GyroBiasCapture");
  microstrain_3dm::GyroBiasCapture srv;


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
