/**
 * @file    microstrain_3dm_gx5_45.h
 * @author  Brian S. Bingham
 * @version 0.1
 *
 * @brief ROS Node for Microstrain
 *
 */


#ifndef _MICROSTRAIN_3DM_GX5_45_TEST_H
#define _MICROSTRAIN_3DM_GX5_45_TEST_H

extern "C" {
#include "mip_sdk.h"
#include "byteswap_utilities.h"
#include "mip_gx4_imu.h"
#include "mip_gx4_45.h"
}
#include <stdio.h>
#include <unistd.h>

#define MIP_SDK_GX4_45_IMU_STANDARD_MODE	0x01
#define MIP_SDK_GX4_45_IMU_DIRECT_MODE	0x02

#define NUM_COMMAND_LINE_ARGUMENTS 3

#define DEFAULT_PACKET_TIMEOUT_MS  1000 //milliseconds

//macro to cause Sleep call to behave as it does for windows
#define Sleep(x) usleep(x*1000.0)

#endif
