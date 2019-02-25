#include "ros/ros.h"
#include "microstrain_3dm_gx5_45/DeviceReport.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "device_report_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_3dm_gx5_45::DeviceReport>("DeviceReport");
  microstrain_3dm_gx5_45::DeviceReport srv;


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
