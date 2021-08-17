cmake_minimum_required(VERSION 2.8.3)
project(ros_mscl_cpp_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    ros_mscl
    cmake_modules
    std_msgs
    std_srvs
)

include_directories(
  /home/microstrain/devel/include
  ${catkin_INCLUDE_DIRS}
)

set(EXAMPLE_SRC_FILES src/listener.cpp)
add_executable(listener_cpp ${EXAMPLE_SRC_FILES})
target_link_libraries(listener_cpp ${catkin_LIBRARIES})

add_dependencies(listener_cpp
  ros_mscl
  mscl_msgs_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

roslint_cpp(${EXAMPLE_SRC_FILES})