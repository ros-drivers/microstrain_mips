cmake_minimum_required(VERSION 2.8.3)
project(microstrain_3dm_gx5_45)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
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
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    nav_msgs
)

###########
## Build ##
###########

# Directories containing MIP SDK header files
set(MIPLIBINC MIPSDK/C/Library/Include)
set(MIPLIBSRC MIPSDK/C/Library/Source)
set(MIPUTILINC MIPSDK/C/Utilities/Include)
set(MIPUTILSRC MIPSDK/C/Utilities/Source)

# The SDK comes with a skeleton for the user-defined functions
# It might be helpful to test compilation against these skeletons
# contains both .c and .h files
# set(MIPUSR "MIPSDK/C/Library/User\ Functions")
#include_directories(include ${MIPLIBINC} ${MIPUTILINC} ${MIPUSR})
include_directories(include/${PROJECT_NAME} ${MIPLIBINC} ${MIPUTILINC} ${catkin_INCLUDE_DIRS})

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

# Libraries
add_library(microstrain_3dm_gx5_45
  src/microstrain_3dm_gx5_45.cpp
  src/mip_sdk_user_functions.c
  ${MIPSRC} 
)

# Executables
add_executable(GX4-45_Test
  src/GX4-45_Test.c
  src/mip_sdk_user_functions.c
  ${MIPSRC} 
)
add_executable(microstrain_3dm_gx5_45_node
  src/microstrain_3dm_gx5_45_node.cpp
)

# Linking
target_link_libraries(GX4-45_Test rt)

target_link_libraries(microstrain_3dm_gx5_45 
  ${catkin_LIBRARIES}
)
target_link_libraries(microstrain_3dm_gx5_45_node 
  microstrain_3dm_gx5_45
  ${catkin_LIBRARIES}
)

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
