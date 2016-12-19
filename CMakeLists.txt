cmake_minimum_required(VERSION 2.8.3)
project(microstrain_3dm_gx5_45)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  #serial
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES microstrain_3dm_gx5_45
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

# Directories containing MIP header files
set(MIPLIBINC MIPSDK/C/Library/Include)
set(MIPLIBSRC MIPSDK/C/Library/Source)
set(MIPUTILINC MIPSDK/C/Utilities/Include)
set(MIPUTILSRC MIPSDK/C/Utilities/Source)

# The SDK comes with a skeleton for the user-defined functions
# It might be helpful to test compilation against these skeletons
# contains both .c and .h files
# set(MIPUSR "MIPSDK/C/Library/User\ Functions")
#include_directories(include ${MIPLIBINC} ${MIPUTILINC} ${MIPUSR})
include_directories(include ${MIPLIBINC} ${MIPUTILINC})
# Include the ROS serial headers
#include_directories(${serial_INCLUDE_DIRS})

set(MIPINC
  ${MIPLIBINC}/mip.h
  ${MIPLIBINC}/mip_sdk_base.h    
  ${MIPLIBINC}/mip_sdk_inteface.h
  ${MIPLIBINC}/mip_sdk_3dm.h   
  ${MIPLIBINC}/mip_sdk_filter.h  
  ${MIPLIBINC}/mip_sdk_system.h
  ${MIPLIBINC}/mip_sdk_ahrs.h  
  ${MIPLIBINC}/mip_sdk_gps.h     
  ${MIPLIBINC}/ring_buffer.h
#  ${MIPUSR}/mip_sdk_user_functions.h
  ${MIPUTILINC}/byteswap_utilities.h
)

set(MIPSRC 
  ${MIPLIBSRC}/mip.c
  ${MIPLIBSRC}/mip_sdk_base.c    
  ${MIPLIBSRC}/mip_sdk_inteface.c
  ${MIPLIBSRC}/mip_sdk_3dm.c   
  ${MIPLIBSRC}/mip_sdk_filter.c  
  ${MIPLIBSRC}/mip_sdk_system.c
  ${MIPLIBSRC}/mip_sdk_ahrs.c  
  ${MIPLIBSRC}/mip_sdk_gps.c     
  ${MIPLIBSRC}/ring_buffer.c
#  ${MIPUSR}/mip_sdk_user_functions.c  
  ${MIPUTILSRC}/byteswap_utilities.c
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(microstrain_3dm_gx5_45
#   src/${PROJECT_NAME}/microstrain_3dm_gx5_45.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(microstrain_3dm_gx5_45 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(microstrain_3dm_gx5_45_node src/microstrain_3dm_gx5_45_node.cpp)
add_executable(GX4-45_Test_node 
  src/GX4-45_Test.c
  src/mip_sdk_user_functions.c
  ${MIPSRC} 
)

#target_link_libraries(GX4-45_Test_node ${serial_LIBRARIES} rt pthread)

#${MIPINC}
## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(microstrain_3dm_gx5_45_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(microstrain_3dm_gx5_45_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS microstrain_3dm_gx5_45 microstrain_3dm_gx5_45_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_microstrain_3dm_gx5_45.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
