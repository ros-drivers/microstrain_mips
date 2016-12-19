/**
 * @file    microstrain_3dm_gx5_45.c 
 * @author  Brian S. Bingham
 * @version 0.1
 *
 * @brief ROS Node for Microstrain
 *
 */

#include "microstrain_3dm_gx5_45.h"

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

 ///
 //Verify the command line arguments
 ///

 if(argc != NUM_COMMAND_LINE_ARGUMENTS)
 {
   printf("Need 3 command line arguments!\n"); // print_command_line_usage();
  return -1;
 }

 const char* portstr = "/dev/ttyUSB0";
 //Convert the arguments
 portstr = argv[1];
 //com_port = atoi(argv[1]);
 baudrate = atoi(argv[2]);

 ///
 //Initialize the interface to the device
 ///
 printf("Attempting to open interface on COM port %d \n",com_port);
 if(mip_interface_init(portstr, baudrate, &device_interface, DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK)
  return -1;

  printf("hi\n");
  return -1;
}
