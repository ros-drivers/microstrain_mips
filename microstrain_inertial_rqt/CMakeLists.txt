cmake_minimum_required(VERSION 3.0.2)
project(microstrain_inertial_rqt)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  rqt_gui
  rqt_gui_py
  std_msgs
  nav_msgs
  geometry_msgs
  microstrain_inertial_msgs
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
    rospy
    tf
    rqt_gui
    rqt_gui_py
    std_msgs
    nav_msgs
    geometry_msgs
    microstrain_inertial_msgs
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(DIRECTORY
  microstrain_inertial_rqt_common/resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/microstrain_inertial_rqt_common
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  microstrain_inertial_rqt_common/plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/microstrain_inertial_rqt_common
)
