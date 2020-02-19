#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

void imuDataCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
	ROS_INFO("Quaternion Orientation:    [%f, %f, %f, %f]", imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w);
	ROS_INFO("Angular Velocity:          [%f, %f, %f]", imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
	ROS_INFO("Linear Acceleration:       [%f, %f, %f]", imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
	
	// add code here to handle incoming IMU data
}

int main(int argc, char** argv)
{
	// register listener to ROS master
	ros::init(argc, argv, "listener");

	// get the device name parameter
	std::string deviceName;
	ros::NodeHandle params("~");
	params.param<std::string>("device", deviceName, "gx5");
	ROS_INFO("Got device param: %s", deviceName.c_str());

	// clear param for future use
	params.deleteParam("device");

	// create the listener node object
	ros::NodeHandle n;

	// subscribe to the imu/data topic
	//Parameters:
	//   topic - namespace (defined in launch file) and topic name
	//   queue size - maximum number of messages to buffer
	//   callback - callback function to handle this data
	ros::Subscriber sub = n.subscribe(("/" + deviceName + "/imu/data"), 3, imuDataCallback);

	// start listening for data
	ros::spin();

	return 0;
}

