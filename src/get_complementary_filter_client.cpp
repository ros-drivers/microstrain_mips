#include "ros/ros.h"
#include "microstrain_3dm/GetComplementaryFilter.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "get_complementary_filter");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_3dm::GetComplementaryFilter>("GetComplementaryFilter");
  microstrain_3dm::GetComplementaryFilter srv;

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
