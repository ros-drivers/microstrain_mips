# microstrain_3dm_gx5_45

## Description

Interface software, including ROS node, for Microstrain 3DM-GX5-45.

The interface makes use of the MIP SDK Version 1.1 from Microstrain to communicate with the device.  Includes the following applications:
 
  * GX4-45_Test : A minimal port of the executable that comes with the SDK including minimial changes in the way the serial port is specified.  Running {rosrun microstrain_3dm_gx5_45 GX4-45_Test /dev/ttyACM0 115200} should run the tests on a unit as a first step.  The program runs through many diagnistics and is a good way to determine compatility with different hardware versions of the 3DM-GXX devices.
  * microstrain_3dm_gx5_45 : ROS node

## Dependencies

 * roscpp
 
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
 
## TODO
 
 * Verify order of quaternions
 
# microstrain_3dm_gx5_45

Documentation for the ROS node - to be moved to ROS wiki when driver is released
 
## Published Topics
 
~gps/fix (sensor_msgs/NavSatFix)

 *Position covariance is populated with diagonals based on reported horizontal and vertical accuracy. 
 * The status.status field is the LLH position data "valid flag"-1.  The valid flag mapping from the 3DM protocol is
  * 0x0001 – Latitude and Longitude Valid
  * 0x0002 – Ellipsoid Height Valid
  * 0x0004 – MSL Height Valid
  * 0x0008 – Horizontal Accuracy Valid
  * 0x0010 – Vertical Accuracy Valid
  * E.g., if all valid, then the status.status field should be 30.
 
~imu/data (sensor_msgs/Imu)
 
~nav/odom (nav_msgs/Odometry)
 
 * Currently the pose.position is the longitude (x), latitude (y) and ellipsoid height (z)
 * pose.covariance and twist.covariance include diagonal elements for position and attitude
 
 ~nav/status (std_msgs/Int16MultiArray)
 
 * Includes three values - see communication protocol for full documentation.
   * filter_state
     * 0x00 – Startup
     * 0x01 – Initialization (see status flags)
     * 0x02 – Running, Solution Valid
     * 0x03 – Running, Solution Error (see status flags)
   * dynamics mode
     * 0x01 – Portable (device default)
     * 0x02 – Automotive 
     * 0x03 – Airborne
   * status_flags
     * See device documentation
 
 ## Services
 
 ~reset_kf (std_srvs/Empty)  TODO
 
 ## Parameters
 
 ~port (string, default: /dev/ttyACM0) 
The serial port to which the device is connected

~baud_rate (integer, default: 115200) 
Baud rate of the sensor. 

~dynamics_mode (int, default: 1)   TODO
     * 0x01 – Portable (device default)
     * 0x02 – Automotive
     * 0x03 – Airborne
     
~declination_source (int, default: 2)   
Possible declination sources:

 * 0x01 – None - device reports magnetic north
 * 0x02 – Internal World Magnetic Model (Default)
 * 0x03 – Manual (see declination parameter)
     
~declination (double, default: 0.23)  NOT YET IMPLEMENTED - Functionality not found in SDK
Sets the declination angle in radians.  Only applies of the declination_source=3.
0.23 radians is +13.27 degrees for Monterey, CA
TODO - verify the sign of this angle

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

~gps_rate (int, default: 1) 
The rates are set as a target value in Hz.  The device accepts a decimation value for each output; the packet rate is base_rate/decimation, where decimation is an intgeter.  The program calculates the decimation to get close the the desired rate, based on polling the sensor for its base rate.

  * Base rate of 4 Hz for 3DM-GX4-45
  
~imu_rate (int, default: 10) 
  
  * Base rate of 500 Hz for 3DM-GX4-45

~nav_rate (int, default: 10) 
  
  * Base rate of 500 Hz for 3DM-GX4-45



  
 
 
 
 
 
 
 
