cmake_minimum_required(VERSION 2.8.3)
project(ros_mscl)

# Locate the common code and messages
set(COMMON_NAME "ros_mscl_common")
set(COMMON_DIR "${${PROJECT_NAME}_SOURCE_DIR}/../${COMMON_NAME}")

# Download and install MSCL
include(${COMMON_DIR}/cmake/download_mscl.cmake)
download_mscl(VERSION "62.0.0")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslint
  message_generation
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  diagnostic_updater
  mscl_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    cmake_modules
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    nav_msgs
    message_runtime
    mscl_msgs
)
###########
## Build ##
###########

# Add the catkin includes
include_directories(${catkin_INCLUDE_DIRS} include/${PROJECT_NAME} ${COMMON_DIR}/include/${COMMON_NAME})

# Set some general CMake flags
add_definitions(-std=c++11)
set(CMAKE_C_FLAGS "-Wno-implicit-function-declaration -Wno-incompatible-pointer-types -Wno-format -fno-builtin-memcpy")

add_library(ros_mscl
  ${COMMON_DIR}/src/microstrain_subscribers.cpp
  ${COMMON_DIR}/src/microstrain_publishers.cpp
  ${COMMON_DIR}/src/microstrain_node_base.cpp
  ${COMMON_DIR}/src/microstrain_services.cpp
  ${COMMON_DIR}/src/microstrain_parser.cpp
  ${COMMON_DIR}/src/microstrain_config.cpp
  src/microstrain_3dm.cpp
)
add_dependencies(${PROJECT_NAME}
  mscl_msgs_generate_messages
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_library(microstrain_diagnostic_updater src/microstrain_diagnostic_updater.cpp)
add_dependencies(microstrain_diagnostic_updater
  mscl_msgs_generate_messages
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# Executables
add_executable(ros_mscl_node
  src/microstrain_3dm_node.cpp
  src/microstrain_diagnostic_updater.cpp
)

# Let the code know if it is being compiled with ROS1 or ROS2
if(DEFINED ENV{ROS_VERSION})
  add_definitions(-DMICROSTRAIN_ROS_VERSION=$ENV{ROS_VERSION})
else()
  message(FATAL_ERROR "ROS_VERSION environment variable is not set.")
endif()

# Include the common code
include_directories(${MSCL_INC_PATH} ${BOOST_INC_PATH})

# Linking
target_link_libraries(ros_mscl
  ${MSCL_LIB_PATH}
  ${catkin_LIBRARIES}
)
target_link_libraries(ros_mscl_node
  ros_mscl
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ros_mscl ros_mscl_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# We also have to install MSCL and it's dependencies so it can be found at runtime
file(GLOB BOOST_LIBS "${MSCL_DIR}/Boost/lib/*")
install(FILES ${BOOST_LIBS} ${MSCL_DIR}/libmscl.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#############
## Testing ##
#############

## Run roslint on only the node files for now
# TODO: Add more files as needed
roslint_cpp(src/microstrain_3dm.cpp src/microstrain_3dm_node.cpp)
