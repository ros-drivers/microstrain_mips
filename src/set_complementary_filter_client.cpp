#include "ros/ros.h"
#include "microstrain_3dm_gx5/SetComplementaryFilter.h"


int main(int argc, char **argv){

  ros::init(argc, argv, "set_complementary_filter_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_3dm_gx5::SetComplementaryFilter>("SetComplementaryFilter");
  microstrain_3dm_gx5::SetComplementaryFilter srv;

  srv.request.north_comp_enable = atoll(argv[1]);
  srv.request.up_comp_enable = atoll(argv[2]);
  srv.request.north_comp_time_const = atoll(argv[3]);
  srv.request.up_comp_time_const = atoll(argv[4]);


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
