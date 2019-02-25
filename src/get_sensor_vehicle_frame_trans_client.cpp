#include "ros/ros.h"
#include "microstrain_3dm_gx5/GetSensorVehicleFrameTrans.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "get_sensor_vehicle_frame_trans");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_3dm_gx5::GetSensorVehicleFrameTrans>("GetSensorVehicleFrameTrans");
  microstrain_3dm_gx5::GetSensorVehicleFrameTrans srv;


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
