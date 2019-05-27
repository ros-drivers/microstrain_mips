#include "ros/ros.h"
#include "microstrain_mips/SetSoftIronMatrix.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "set_soft_iron_matrix_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_mips::SetSoftIronMatrix>("SetSoftIronMatrix");
  microstrain_mips::SetSoftIronMatrix srv;

  srv.request.soft_iron_1.x = atoll(argv[1]);
  srv.request.soft_iron_1.y = atoll(argv[2]);
  srv.request.soft_iron_1.z = atoll(argv[3]);
  srv.request.soft_iron_2.x = atoll(argv[4]);
  srv.request.soft_iron_2.y = atoll(argv[5]);
  srv.request.soft_iron_2.z = atoll(argv[6]);
  srv.request.soft_iron_3.x = atoll(argv[7]);
  srv.request.soft_iron_3.y = atoll(argv[8]);
  srv.request.soft_iron_3.z = atoll(argv[9]);


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
