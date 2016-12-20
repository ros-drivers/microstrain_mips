/**
 * @file    microstrain_3dm_gx5_45.c 
 * @author  Brian S. Bingham
 * @version 0.1
 *
 * @brief ROS Node for Microstrain
 *
 */

#include "microstrain_3dm_gx5_45.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

/*
// Make C functions callable
extern "C" {
  u16 mip_interface_init(const char *portstr, u32 baudrate, mip_interface *device_interface, u32 packet_timeout_val);
};
*/

/* Globals */

u8 enable_data_stats_output = 0;

//The primary device interface structure
mip_interface device_interface;

//Packet Counters (valid, timeout, and checksum errors)
u32 filter_valid_packet_count  = 0;
u32 ahrs_valid_packet_count = 0;
u32 gps_valid_packet_count  = 0;

u32 filter_timeout_packet_count  = 0;
u32 ahrs_timeout_packet_count = 0;
u32 gps_timeout_packet_count  = 0;

u32 filter_checksum_error_packet_count  = 0;
u32 ahrs_checksum_error_packet_count = 0;
u32 gps_checksum_error_packet_count  = 0;

//Example data field storage

//AHRS
mip_ahrs_scaled_gyro  curr_ahrs_gyro;
mip_ahrs_scaled_accel curr_ahrs_accel;
mip_ahrs_scaled_mag   curr_ahrs_mag;

//GPS
mip_gps_llh_pos curr_llh_pos;
mip_gps_ned_vel curr_ned_vel;
mip_gps_time    curr_gps_time;

//FILTER
mip_filter_llh_pos               curr_filter_pos;
mip_filter_ned_velocity          curr_filter_vel;
mip_filter_attitude_euler_angles curr_filter_angles;

// ROS Globals
ros::Publisher gps_pub;
sensor_msgs::NavSatFix gps_msg;

/* Main */
int main(int argc, char **argv)
{

 u32 com_port, baudrate;
 base_device_info_field device_info;
 u8  temp_string[20] = {0};
 u32 bit_result;
 u8  enable = 1;
 u8  data_stream_format_descriptors[10];
 u16 data_stream_format_decimation[10];
 u8  data_stream_format_num_entries = 0;
 u8  readback_data_stream_format_descriptors[10] = {0};
 u16 readback_data_stream_format_decimation[10]  = {0};
 u8  readback_data_stream_format_num_entries     =  0;
 u16 base_rate = 0;
 u16 device_descriptors[128]  = {0};
 u16 device_descriptors_size  = 128*2;
 s16 i;
 u16 j;
 u8  com_mode = 0;
 u8  readback_com_mode = 0;
 float angles[3]             = {0};
 float readback_angles[3]    = {0};
 float offset[3]             = {0};
 float readback_offset[3]    = {0};
 float hard_iron[3]          = {0};
 float hard_iron_readback[3] = {0};
 float soft_iron[9]          = {0};
 float soft_iron_readback[9] = {0};
 u8  dynamics_mode           = 0;
 u8  readback_dynamics_mode  = 0;
 u16 estimation_control   = 0, estimation_control_readback = 0;
 u8  gps_source     = 0;
 u8  heading_source = 0;
 u8  auto_init      = 0;
 float noise[3]          = {0};
 float readback_noise[3] = {0};
 float beta[3]                 = {0};
 float readback_beta[3]        = {0};
 mip_low_pass_filter_settings filter_settings;
 float bias_vector[3]		   = {0};
 u16 duration = 0;
 gx4_imu_diagnostic_device_status_field imu_diagnostic_field;
 gx4_imu_basic_status_field imu_basic_field;
 gx4_45_diagnostic_device_status_field diagnostic_field;
 gx4_45_basic_status_field basic_field;
 mip_filter_external_gps_update_command external_gps_update;
 mip_filter_external_heading_update_command external_heading_update;
 mip_filter_zero_update_command zero_update_control, zero_update_readback;
 mip_filter_external_heading_with_time_command external_heading_with_time;
 mip_complementary_filter_settings comp_filter_command, comp_filter_readback;

 u8  declination_source_command, declination_source_readback;

 mip_filter_accel_magnitude_error_adaptive_measurement_command        accel_magnitude_error_command, accel_magnitude_error_readback;
 mip_filter_magnetometer_magnitude_error_adaptive_measurement_command mag_magnitude_error_command, mag_magnitude_error_readback;
 mip_filter_magnetometer_dip_angle_error_adaptive_measurement_command mag_dip_angle_error_command, mag_dip_angle_error_readback;

 // ROS Setup
 ros::init(argc,argv, "microstrain_3dm_gx5_45");
 ros::NodeHandle node;
 ros::NodeHandle private_nh("~");

 gps_pub = node.advertise<sensor_msgs::NavSatFix>("microstrain_gps",100);
 

 // ROS Parameters
 std::string port;
 int baud;
 private_nh.param("port", port, std::string("/dev/ttyACM0"));
 private_nh.param("baudrate",baud,115200);

 baudrate = (u32)baud; //115200;

 //Initialize the interface to the device
 ROS_INFO("Attempting to open serial port <%s> at <%d> \n",
	  port.c_str(),baudrate);
 if(mip_interface_init(port.c_str(), baudrate, &device_interface, DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK){
   ROS_ERROR("Couldn't open port!");
 }

 float dT=1.5;  // common sleep time after communications
 /* Setup and test Comms */
 // Put device into standard mode
 ROS_INFO("Put device into standard comms mode");
 device_descriptors_size  = 128*2;
 com_mode = MIP_SDK_GX4_45_IMU_STANDARD_MODE;
 while(mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &com_mode) != MIP_INTERFACE_OK){}
 //Verify device mode setting
 while(mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_READ, &com_mode) != MIP_INTERFACE_OK){}
 ros::Duration(dT).sleep();
 if(com_mode != MIP_SDK_GX4_45_IMU_STANDARD_MODE)
   {
     ROS_ERROR("Appears we didn't get into standard mode!");
   }

 // Put into idle mode
 ROS_INFO("Idling Device");
 while(mip_base_cmd_idle(&device_interface) != MIP_INTERFACE_OK){}
 ros::Duration(dT).sleep();

 // Get supported descriptors
 /*
  while(mip_base_cmd_get_device_supported_descriptors(&device_interface, (u8*)device_descriptors, &device_descriptors_size) != MIP_INTERFACE_OK){}

  std::printf("\n\nSupported descriptors:\n\n");

  for(i=0; i< device_descriptors_size/2; i++)
  {
    std::printf("Descriptor Set: %02x, Descriptor: %02x\n", device_descriptors[i] >> 8, device_descriptors[i]&0xFF);
   Sleep(100);
  }

  std::printf("\n\n");
  Sleep(1500);
 */


 // Setup callbacks
 if(mip_interface_add_descriptor_set_callback(&device_interface, MIP_FILTER_DATA_SET, NULL, &filter_packet_callback) != MIP_INTERFACE_OK)
   return -1;
  
 if(mip_interface_add_descriptor_set_callback(&device_interface, MIP_AHRS_DATA_SET, NULL, &ahrs_packet_callback) != MIP_INTERFACE_OK)
   return -1;

 if(mip_interface_add_descriptor_set_callback(&device_interface, MIP_GPS_DATA_SET, NULL, &gps_packet_callback) != MIP_INTERFACE_OK)
   return -1;

 // Get rates
 while(mip_3dm_cmd_get_ahrs_base_rate(&device_interface, &base_rate) != MIP_INTERFACE_OK){}
 ROS_INFO("AHRS Base Rate => %d Hz", base_rate);
 ros::Duration(dT).sleep();

 while(mip_3dm_cmd_get_gps_base_rate(&device_interface, &base_rate) != MIP_INTERFACE_OK){}
 ROS_INFO("GPS Base Rate => %d Hz", base_rate);
 ros::Duration(dT).sleep();
 
 while(mip_3dm_cmd_get_filter_base_rate(&device_interface, &base_rate) != MIP_INTERFACE_OK){}
 ROS_INFO("FILTER Base Rate => %d Hz", base_rate);
  ros::Duration(dT).sleep();

 // Set message formats
 enable_data_stats_output = 1;
 ROS_INFO("Setting the AHRS message format");
 data_stream_format_descriptors[0] = MIP_AHRS_DATA_ACCEL_SCALED;
 data_stream_format_descriptors[1] = MIP_AHRS_DATA_GYRO_SCALED;
 data_stream_format_decimation[0]  = 0x32;
 data_stream_format_decimation[1]  = 0x32;
 data_stream_format_num_entries = 2;
 while(mip_3dm_cmd_ahrs_message_format(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries, data_stream_format_descriptors, data_stream_format_decimation) != MIP_INTERFACE_OK){}
 ros::Duration(dT).sleep();
 ROS_INFO("Poll AHRS data to verify");
 while(mip_3dm_cmd_poll_ahrs(&device_interface, MIP_3DM_POLLING_ENABLE_ACK_NACK, data_stream_format_num_entries, data_stream_format_descriptors) != MIP_INTERFACE_OK){}
 ros::Duration(dT).sleep();
 ROS_INFO(" ");
 

 ROS_INFO("Setting GPS stream format");
 data_stream_format_descriptors[0] = MIP_GPS_DATA_LLH_POS;
 data_stream_format_descriptors[1] = MIP_GPS_DATA_NED_VELOCITY;
 data_stream_format_descriptors[2] = MIP_GPS_DATA_GPS_TIME;
 data_stream_format_decimation[0]  = 0x01; //0x04;
 data_stream_format_decimation[1]  = 0x01;
 data_stream_format_decimation[2]  = 0x01;
 data_stream_format_num_entries = 3;
 while(mip_3dm_cmd_gps_message_format(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries,data_stream_format_descriptors, data_stream_format_decimation) != MIP_INTERFACE_OK){}
 ros::Duration(dT).sleep();

 ROS_INFO("Setting Filter stream format");
 data_stream_format_descriptors[0] = MIP_FILTER_DATA_LLH_POS;
 data_stream_format_descriptors[1] = MIP_FILTER_DATA_NED_VEL;
 data_stream_format_descriptors[2] = MIP_FILTER_DATA_ATT_EULER_ANGLES;
 data_stream_format_decimation[0]  = 0x32;
 data_stream_format_decimation[1]  = 0x32;
 data_stream_format_decimation[2]  = 0x32;
 data_stream_format_num_entries = 3;
 while(mip_3dm_cmd_filter_message_format(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries,data_stream_format_descriptors, data_stream_format_decimation) != MIP_INTERFACE_OK){}
 ros::Duration(dT).sleep();
 ROS_INFO("Poll filter data to test stream");
 while(mip_3dm_cmd_poll_filter(&device_interface, MIP_3DM_POLLING_ENABLE_ACK_NACK, data_stream_format_num_entries, data_stream_format_descriptors) != MIP_INTERFACE_OK){}
 ros::Duration(dT).sleep();
 ROS_INFO(" ");

 // Set dynamics mode
 //dynamics_mode = 1;
 //while(mip_filter_vehicle_dynamics_mode(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &dynamics_mode) != MIP_INTERFACE_OK){}
 // Default mode
 ROS_INFO("Setting default dynamics mode");
 while(mip_filter_vehicle_dynamics_mode(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) != MIP_INTERFACE_OK){}
 ros::Duration(dT).sleep();

 // Default auto-init
 ROS_INFO("Setting default auto-init");
  while(mip_filter_auto_initialization(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) != MIP_INTERFACE_OK){}
  ros::Duration(dT).sleep();

 // Reset filter
 ROS_INFO("Reset filter");
 while(mip_filter_reset_filter(&device_interface) != MIP_INTERFACE_OK){}
 ros::Duration(dT).sleep();

 // Enable Data streams
 ROS_INFO("Enabling AHRS stream");
 enable = 0x01;
 while(mip_3dm_cmd_continuous_data_stream(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_AHRS_DATASTREAM, &enable) != MIP_INTERFACE_OK){}
 ros::Duration(dT).sleep();

 ROS_INFO("Enabling Filter stream");
 enable = 0x01;
 while(mip_3dm_cmd_continuous_data_stream(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_INS_DATASTREAM, &enable) != MIP_INTERFACE_OK){}
 ros::Duration(dT).sleep();

 ROS_INFO("Enabling GPS stream");
 enable = 0x01;
while(mip_3dm_cmd_continuous_data_stream(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_GPS_DATASTREAM, &enable) != MIP_INTERFACE_OK){}
 ros::Duration(dT).sleep();

 // Loop
 enable_data_stats_output = 1;

 ros::Rate r(1000);  // Rate in Hz
 while (ros::ok()){
   //Update the parser (this function reads the port and parses the bytes
   mip_interface_update(&device_interface);
   r.sleep();
   //ROS_INFO("Spinning");
 }
 
 return 0;
}


void filter_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
{
 mip_field_header *field_header;
 u8               *field_data;
 u16              field_offset = 0;

 //ROS_INFO("Filter callback");
 //The packet callback can have several types, process them all
 switch(callback_type)
 {
  ///
  //Handle valid packets
  ///

  case MIP_INTERFACE_CALLBACK_VALID_PACKET:
  {
   filter_valid_packet_count++;

   ///
   //Loop through all of the data fields
   ///

   while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
   {

    ///
    // Decode the field
    ///

    switch(field_header->descriptor)
    {
     ///
     // Estimated LLH Position
     ///

     case MIP_FILTER_DATA_LLH_POS:
     {
      memcpy(&curr_filter_pos, field_data, sizeof(mip_filter_llh_pos));

      //For little-endian targets, byteswap the data field
      mip_filter_llh_pos_byteswap(&curr_filter_pos);

     }break;

     ///
     // Estimated NED Velocity
     ///

     case MIP_FILTER_DATA_NED_VEL:
     {
      memcpy(&curr_filter_vel, field_data, sizeof(mip_filter_ned_velocity));

      //For little-endian targets, byteswap the data field
      mip_filter_ned_velocity_byteswap(&curr_filter_vel);

     }break;

     ///
     // Estimated Attitude, Euler Angles
     ///

     case MIP_FILTER_DATA_ATT_EULER_ANGLES:
     {
      memcpy(&curr_filter_angles, field_data, sizeof(mip_filter_attitude_euler_angles));

      //For little-endian targets, byteswap the data field
      mip_filter_attitude_euler_angles_byteswap(&curr_filter_angles);

     }break;

     default: break;
    }
   }
  }break;


  ///
  //Handle checksum error packets
  ///

  case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
  {
   filter_checksum_error_packet_count++;
  }break;

  ///
  //Handle timeout packets
  ///

  case MIP_INTERFACE_CALLBACK_TIMEOUT:
  {
   filter_timeout_packet_count++;
  }break;
  default: break;
 }

 print_packet_stats();
}


////////////////////////////////////////////////////////////////////////////////
//
// AHRS Packet Callback
//
////////////////////////////////////////////////////////////////////////////////

void ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
{
 mip_field_header *field_header;
 u8               *field_data;
 u16              field_offset = 0;

 //The packet callback can have several types, process them all
 switch(callback_type)
 {
  ///
  //Handle valid packets
  ///

  case MIP_INTERFACE_CALLBACK_VALID_PACKET:
  {
   ahrs_valid_packet_count++;

   ///
   //Loop through all of the data fields
   ///

   while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
   {

    ///
    // Decode the field
    ///

    switch(field_header->descriptor)
    {
     ///
     // Scaled Accelerometer
     ///

     case MIP_AHRS_DATA_ACCEL_SCALED:
     {
      memcpy(&curr_ahrs_accel, field_data, sizeof(mip_ahrs_scaled_accel));

      //For little-endian targets, byteswap the data field
      mip_ahrs_scaled_accel_byteswap(&curr_ahrs_accel);

     }break;

     ///
     // Scaled Gyro
     ///

     case MIP_AHRS_DATA_GYRO_SCALED:
     {
      memcpy(&curr_ahrs_gyro, field_data, sizeof(mip_ahrs_scaled_gyro));

      //For little-endian targets, byteswap the data field
      mip_ahrs_scaled_gyro_byteswap(&curr_ahrs_gyro);

     }break;

     ///
     // Scaled Magnetometer
     ///

     case MIP_AHRS_DATA_MAG_SCALED:
     {
      memcpy(&curr_ahrs_mag, field_data, sizeof(mip_ahrs_scaled_mag));

      //For little-endian targets, byteswap the data field
      mip_ahrs_scaled_mag_byteswap(&curr_ahrs_mag);

     }break;

     default: break;
    }
   }
  }break;

  ///
  //Handle checksum error packets
  ///

  case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
  {
   ahrs_checksum_error_packet_count++;
  }break;

  ///
  //Handle timeout packets
  ///

  case MIP_INTERFACE_CALLBACK_TIMEOUT:
  {
   ahrs_timeout_packet_count++;
  }break;
  default: break;
 }

 print_packet_stats();
}


////////////////////////////////////////////////////////////////////////////////
//
// GPS Packet Callback
//
////////////////////////////////////////////////////////////////////////////////

void gps_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
{
 mip_field_header *field_header;
 u8               *field_data;
 u16              field_offset = 0;

 //The packet callback can have several types, process them all
 switch(callback_type)
 {
  ///
  //Handle valid packets
  ///

  case MIP_INTERFACE_CALLBACK_VALID_PACKET:
  {
   gps_valid_packet_count++;

   ///
   //Loop through all of the data fields
   ///

   while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
   {

    ///
    // Decode the field
    ///

    switch(field_header->descriptor)
    {
     ///
     // LLH Position
     ///

     case MIP_GPS_DATA_LLH_POS:
     {
      memcpy(&curr_llh_pos, field_data, sizeof(mip_gps_llh_pos));

      //For little-endian targets, byteswap the data field
      mip_gps_llh_pos_byteswap(&curr_llh_pos);

     }break;

     ///
     // NED Velocity
     ///

     case MIP_GPS_DATA_NED_VELOCITY:
     {
      memcpy(&curr_ned_vel, field_data, sizeof(mip_gps_ned_vel));

      //For little-endian targets, byteswap the data field
      mip_gps_ned_vel_byteswap(&curr_ned_vel);

     }break;

     ///
     // GPS Time
     ///

     case MIP_GPS_DATA_GPS_TIME:
     {
      memcpy(&curr_gps_time, field_data, sizeof(mip_gps_time));

      //For little-endian targets, byteswap the data field
      mip_gps_time_byteswap(&curr_gps_time);

     }break;

     default: break;
    }
   }
  }break;


  ///
  //Handle checksum error packets
  ///

  case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
  {
   gps_checksum_error_packet_count++;
  }break;

  ///
  //Handle timeout packets
  ///

  case MIP_INTERFACE_CALLBACK_TIMEOUT:
  {
   gps_timeout_packet_count++;
  }break;
  default: break;
 }

 // Publish the message
 gps_pub.publish(gps_msg);

 print_packet_stats();
}

void print_packet_stats()
{
 if(enable_data_stats_output)
 {
  printf("\r%u FILTER (%u errors)    %u AHRS (%u errors)    %u GPS (%u errors) Packets", filter_valid_packet_count,  filter_timeout_packet_count + filter_checksum_error_packet_count,
                                                                                      ahrs_valid_packet_count, ahrs_timeout_packet_count + ahrs_checksum_error_packet_count,
                                                                                      gps_valid_packet_count,  gps_timeout_packet_count + gps_checksum_error_packet_count);
 }
}
