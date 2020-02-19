cmake_minimum_required(VERSION 2.8.3)
project(ros_mscl_cpp_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(listener_cpp src/listener.cpp)
target_link_libraries(listener_cpp ${catkin_LIBRARIES})
