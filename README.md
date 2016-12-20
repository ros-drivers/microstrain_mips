# microstrain_3dm_gx5_45

## Description

Interface software, including ROS node, for Microstrain 3DM-GX5-45.

The interface makes use of the MIP SDK Version 1.1 from Microstrain to communicate with the device.  Includes the following applications:
 
  * GX4-45_Test : A minimal port of the executable that comes with the SDK including minimial changes in the way the serial port is specified.  Running {rosrun microstrain_3dm_gx5_45 GX4-45_Test /dev/ttyACM0 115200} should run the tests on a unit as a first step.  The program runs through many diagnistics and is a good way to determine compatility with different hardware versions of the 3DM-GXX devices.
  * microstrain_3dm_gx5_45 : ROS node

## Dependencies

 * serial: https://github.com/wjwwood/serial  - for now we build this from source in our catkin_ws
 
## Build instructions.  

The concept is to build this drive along with the Microstrain SDK (Version 1.1).  Currently the repo contains the SDK files,
but in the future, it might make sense to not include them - or to substitute a newer version.  
Here are the steps I took to build the code...

 * Dowload the zip archive "MIP C Code Sample for Windows and Linux Version 1.1" from the Microstrain website
 * Extract the archive into the directory "MIPSDK".  The first directory in MIPSDK should be "C"
 * In C/Library/User Functions/mip_sdk_user_functions.c, comment out the line "#include <windows.h>
 * cd catkin_sw
 * catkin_make
 
## Dev Notes
 
 The mip_sdk_user_functions are C functions that need to be called by various parts of the SDK.  The main purpose of these functions is to implement the platform-specific serial (RS232) port elements.  The prototype serial port open function takes the COM number as an integer input - which is clunky for Linux serial ports.  Changed this to take a string defining the port (e.g., /dev/ttyS0), but this necessitated also modifying the mip_sdk_interface.[ch] files, since this is what is called by the application - changed the mip_interface_init function to accept a string argument for specifying the port.
 
 # microstrain_3dm_gx5_45
 Documentation for the ROS node - to be moved to ROS wiki when driver is released
 
 ## Published Topics
 
 ~gps/fix (sensor_msgs/NavSatFix)
 
 ~imu/data (sensor_msgs/Imu)
 
 ~nav/odom (nav_msgs/Odometry)
 
 ~nav/status (std_msgs/Int16MultiArray)
 
 
 ## Services
 
 ~reset_kf (std_srvs/Empty)  TODO
 
 ## Parameters
 
 ~port (string, default: /dev/ttyACM0) 
The serial port to which the device is connected

~baud_rate (integer, default: 115200) 
Baud rate of the sensor. 

~declination (double, default: 3.8)   TODO
Magnetic declination for given area. 

~gps_frame_id (string, default: world)
Value for the frame_id field of the 

~child_frame_id (string, default: /base_footprint)   TODO
Odometry data will be published with this child_frame_id.

~publish_pose (bool, default: true) 
Sets if ~imu/pose should be advertised or not. 

~publish_imu (bool, default: true) 
Sets if ~imu/data should be advertised or not. 

~publish_gps (bool, default: true) 
Sets if ~gps/fix should be advertised or not. 
  
 
 
 
 
 
 
 
