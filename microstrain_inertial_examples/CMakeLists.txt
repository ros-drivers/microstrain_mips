cmake_minimum_required(VERSION 2.8.3)
project(microstrain_inertial_examples)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslint
  microstrain_inertial_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    microstrain_inertial_msgs
    cmake_modules
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# CPP Listener
set(LISTENER_CPP listener_cpp)
set(LISTENER_CPP_SRC_FILES src/listener.cpp)
add_executable(${LISTENER_CPP} ${LISTENER_CPP_SRC_FILES})
target_link_libraries(${LISTENER_CPP} ${catkin_LIBRARIES})
add_dependencies(${LISTENER_CPP}
  microstrain_inertial_msgs_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# Python Listener
catkin_install_python(PROGRAMS scripts/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install the C++ Examples
install(TARGETS ${LISTENER_CPP}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install the launch files
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

roslint_cpp(${LISTENER_CPP_SRC_FILES})