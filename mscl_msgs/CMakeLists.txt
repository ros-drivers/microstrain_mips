cmake_minimum_required(VERSION 2.8.3)
project(mscl_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

# Locate the common code and messages
set(COMMON_NAME "ros_mscl_common")
set(COMMON_DIR "${${PROJECT_NAME}_SOURCE_DIR}/../${COMMON_NAME}")

# Generate Service Files
set(SRV_DIR "${COMMON_DIR}/srv")
FILE(GLOB SRV_FILES RELATIVE "${SRV_DIR}"
    "${SRV_DIR}/*.srv")
add_service_files(
  DIRECTORY ${SRV_DIR}
  FILES ${SRV_FILES}
)

# Generate Message Files
set(MSG_DIR "${COMMON_DIR}/msg")
FILE(GLOB MSG_FILES RELATIVE "${MSG_DIR}"
    "${MSG_DIR}/*.msg")
add_message_files(
  DIRECTORY ${MSG_DIR}
  FILES ${MSG_FILES}
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
)
