#include "ros/ros.h"
#include "microstrain_3dm/GetSensorVehicleFrameOffset.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "get_sensor_vehicle_frame_offset");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_3dm::GetSensorVehicleFrameOffset>("GetSensorVehicleFrameOffset");
  microstrain_3dm::GetSensorVehicleFrameOffset srv;

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
