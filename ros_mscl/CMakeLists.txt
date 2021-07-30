cmake_minimum_required(VERSION 2.8.3)
project(ros_mscl)

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

# Install MSCL from github
set(MSCL_GITHUB_ORG "LORD-MicroStrain")
set(MSCL_GITHUB_REPO "MSCL")
set(MSCL_GITHUB_TAG "v62.0.0")
set(MSCL_GITHUB_ARTIFACT "c++-mscl_62.0.0_amd64.deb")

# Get the release ID for the version we are trying to download
set(MSCL_LIST_REPO_URL "https://api.github.com/repos/${MSCL_GITHUB_ORG}/${MSCL_GITHUB_REPO}/releases/tags/${MSCL_GITHUB_TAG}")
execute_process(
  COMMAND ${CMAKE_COMMAND} -E env /bin/bash -c "curl -fsSl '${MSCL_LIST_REPO_URL}' | jq '.assets[] | select(.name==\"${MSCL_GITHUB_ARTIFACT}\") | .url' | sed 's/\"//g'"
  OUTPUT_VARIABLE MSCL_DOWNLOAD_LINK
)

# Download the deb file
set(MSCL_DEB_ARTIFACT "${PROJECT_BINARY_DIR}/${MSCL_GITHUB_ARTIFACT}")
execute_process(
  COMMAND ${CMAKE_COMMAND} -E env /bin/bash -c "curl -fsSLJo ${MSCL_DEB_ARTIFACT} -H 'Accept: application/octet-stream' ${MSCL_DOWNLOAD_LINK}"
)

# Extract the deb file into the workspace
get_filename_component(MSCL_DEB_EXTRACTED_DIR_NAME "${MSCL_GITHUB_ARTIFACT}" NAME_WE)
set(MSCL_DEB_EXTRACTED_DIR "${PROJECT_BINARY_DIR}/${MSCL_DEB_EXTRACTED_DIR_NAME}")
execute_process(
  COMMAND ${CMAKE_COMMAND} -E env /bin/bash -c "dpkg-deb -x '${MSCL_DEB_ARTIFACT}' '${MSCL_DEB_EXTRACTED_DIR}'"
)
set(MSCL_DIR "${MSCL_DEB_EXTRACTED_DIR}/usr/share/c++-mscl")


find_library(MSCL_LIB_PATH NAMES libmscl.so PATHS "${MSCL_DIR}" DOC "MSCL Library" NO_DEFAULT_PATH)
find_library(BOOST_LIB_PATH NAMES libboost_chrono.so PATHS "${MSCL_DIR}/Boost/lib" NO_DEFAULT_PATH)
set(MSCL_INC_PATH "${MSCL_DIR}/source")
set(BOOST_INC_PATH "${MSCL_DIR}/Boost/include")

add_definitions(-std=c++11)

set(CMAKE_C_FLAGS "-Wno-implicit-function-declaration -Wno-incompatible-pointer-types -Wno-format -fno-builtin-memcpy")

FILE(GLOB SRV_FILES RELATIVE "${${PROJECT_NAME}_SOURCE_DIR}/srv"
    "${${PROJECT_NAME}_SOURCE_DIR}/srv/*.srv")

add_service_files(
  FILES
  ${SRV_FILES}
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

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

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

include_directories(include/${PROJECT_NAME} ${MIPLIBINC} ${MIPUTILINC} ${MSCL_INC_PATH} ${MSCL_LIB} ${MSCL_INC} ${catkin_INCLUDE_DIRS})

add_library(ros_mscl
  src/microstrain_3dm.cpp
)
add_dependencies(${PROJECT_NAME}
  mscl_msgs_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_library(microstrain_diagnostic_updater src/microstrain_diagnostic_updater.cpp)
add_dependencies(microstrain_diagnostic_updater
  mscl_msgs_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# Executables

add_executable(ros_mscl_node
  src/microstrain_3dm_node.cpp
  src/microstrain_diagnostic_updater.cpp
)

add_dependencies(ros_mscl_node
  ros_mscl_gencpp
  mscl_msgs_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

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
