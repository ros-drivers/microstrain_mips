^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_mscl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.4 (2021-05-19)
------------------
* MSCL Version now reported when node starts
* Aiding measurement summary added for each GNSS (GQ7 only) 

1.1.3 (2021-04-05)
------------------
* Added a flag to report values with-respect-to the ENU frame instead of the device-native NED frame.  This affects all reported position, velocity, and attitude values.
* Removed user-settable frame ids so that the code can correctly determine if they should include "NED" or "ENU" frame designators
* Moved sensor2vehicle transformation code outside of publish_filter check as it applies to IMU data even if not publishing filter data


1.1.2 (2021-02-24)
------------------
* BREAKING CHANGE: Switched order of latitude and longitude in /nav/odom message to be correct (now Latitude, Longitude, Height)
* Added support for external heading messages
* Added support for relative position ouput on /device_name/nav/relative_pos/odom (GQ7 only)
* Removed service clients as feedback has been that they are not used and clutter driver code
* Fixed crash issue identified in https://github.com/LORD-MicroStrain/ROS-MSCL/issues/35

1.1.1 (2020-12-08)
------------------
* Added support for raw binary file output (used for support issues)
* Added support for RTK status message received on data class GNSS3 (GQ7 only)
* Added factory streaming setup for devices that support this command (GQ7 currently)

1.1.0 (2020-09-14)
------------------
* Added preliminary support for GQ7
* Changed "GPS" topic to "GNSS1" and added "GNSS2" topic in support of GQ7
* Refactored driver code for more explicit variable names
* Changed initialization procedure to setup device first, then topics, then services
* Modified example launch file with more specific parameter names


1.0.0 (2020-02-13)
------------------
* Added mscl support
* Removed MIPSDK and utility functions in mip_sdk_user_functions
* Changed name of node and package to ros_mscl
* Added device feature detection to improve compatibility with a greater range of devices
* Added Parker Hannifin Corp to maintainers

0.0.3 (2019-08-05)
------------------
* Made diagnostic_updater build dep as well.
* Merge pull request `#21 <https://github.com/ros-drivers/microstrain_mips/issues/21>`_ from samkys/cleanup
  Cleanup
* Add roslint and cleaned up files accordingly.
* Cleaned up indentation levels, removed tabs and replaced with spaces, and updated curly brace locations according to: http://wiki.ros.org/CppStyleGuide section 6.
* Cleanup that was forgotten in last commit.
* Added static IMU message covariance population via parameters.
* Contributors: Sam, Tony Baltovski

0.0.2 (2019-05-28)
------------------
* Merge pull request `#18 <https://github.com/ros-drivers/microstrain_mips/issues/18>`_ from ljazzal/master
  Harmonized package name throughout the source code
* renamed remaining "microstrain_3dm_node"
* renamed microstrain_3dm to microstrain_mips
* Merge pull request `#16 <https://github.com/ros-drivers/microstrain_mips/issues/16>`_ from wxmerkt/wxm-fix-compilation
  Fix compilation (set exported targets dependency)
* Fix compilation (set exported targets dependency)
* Merge pull request `#15 <https://github.com/ros-drivers/microstrain_mips/issues/15>`_ from ljazzal/master
  Enabled diagnostic updater for microstrain ROS driver extension
* Merge pull request `#14 <https://github.com/ros-drivers/microstrain_mips/issues/14>`_ from wxmerkt/wxm-fix-install
  Do not install udev
* changed namespace of microstrain sensor node
* Do not install udev
  Fixes build in install workspace after `#11 <https://github.com/ros-drivers/microstrain_mips/issues/11>`_
* fixed status callback
* minor fixes
* minor fixes: naming convention
* Changed default port name
* potentially fixing diagnostic updater
* Replaced several services with Trigger srv
* Added comments
* Minor changes to default settings
* edited device status function
* Edited cmakelists
* Adding to readme
* Adding to readme
* Testing readme additions
* Testing readme additions
* Testing readme additions
* deleted extra variables
* Diagnostic reporting working on GX5-25
* Latest code
* Added diagnostics, commenting
* Added gx5-45 files
* Turned basic status into mssg
* having trouble with device status function
* Added srvs for functions that were previously not working.
* added more services.
* Added basic and diagnostic status reporting through minor changes to sdk.
* Changed names to generic GX5.
* added srvs to change settings.
* minor edits
* Merge pull request `#1 <https://github.com/ros-drivers/microstrain_mips/issues/1>`_ from shreyasubbu/minor-changes
  Minor changes for SetBias service.
* Minor changes for SetBias service.
* can compile, but does not work
* 1st commit
* Merge pull request `#11 <https://github.com/ros-drivers/microstrain_mips/issues/11>`_ from ros-drivers/udev
  Installed udev rules for release.
  Great - thank you.
* Installed udev rules for release.
* Merge pull request `#8 <https://github.com/ros-drivers/microstrain_mips/issues/8>`_ from pvechersky/feature/launch_args
  Adding arguments to launch files
* Merge pull request `#10 <https://github.com/ros-drivers/microstrain_mips/issues/10>`_ from wxmerkt/master
  Turn off MIPSDK compile time warnings.
  Thank you!
* Turn off MIPSDK compile time warnings.
* Removing hardware-specific kf and pioneer launch files
* Adding arguments to launch files, making microstrain.launch more generic
* Merge pull request `#7 <https://github.com/ros-drivers/microstrain_mips/issues/7>`_ from pvechersky/feature/package_installation
  Adding the installation step to CMakeLists.txt
* Adding the installation step to CMakeLists.txt
* Merge branch 'gx25'
* Merge pull request `#3 <https://github.com/ros-drivers/microstrain_mips/issues/3>`_ from clearpathrobotics/gx25
  Gx25 launch file and udev rule
* Create 99-microstrain.rules
  Creates a symlink in /dev when a Microstrain device is connected.
* Update microstrain_25.launch
  removed references to GX4
  now looks for the "/dev/microstrain" symlink created by the udev rule
  changed the frame_ids to more conventional or useful names
* bumped up spin rate
* Merge branch 'master' into gx25
* adding pioneer launch
* Set spin rate as a function of message updates.  Addressed sigterm issue
* prototype driver for -25
* adding kf launch file
* adding kf specific launch file
* futzing with quat
* Changing conversion from MIP quaterinion to tf2
* Changing conversion from MIP quaterinion to tf2
* Changing conversion from MIP quaterinion to tf2
* Changing conversion from MIP quaterinion to tf2
* Changing conversion from MIP quaterinion to tf2
* Changing conversion from MIP quaterinion to tf2
* adding debug for filter state
* adding debug for filter state
* adding debug for filter state
* adding debug for filter state
* adding debug for filter state
* bug fix
* adding kf launch file
* debugging on kf
* debugging on kf
* resolving conflicts
* docs
* cleaning package.xml
* adding licensing information
* adding wiki file
* adding explicit link to cmake for hydro
* Merge branch 'master' of github.com:bsb808/microstrain_3dm_gx5_45
* adding dependency
* Update README.md
* adding a transform and cleaning up dependencies
* Update README.md
* Merge branch 'master' of github.com:bsb808/microstrain_3dm_gx5_45
* Update README.md
* incremental progress on cleaning up settings
* reorganizing ode and includes - separating library and node executable.
* reorganized include directory
* Update README.md
* added many functions, parameters and a reset_kf service
* Update README.md
* Update README.md
* Update README.md
* publishing up to 500 Hz!
* adding a publish test
* Merge branch 'master' of github.com:bsb808/microstrain_3dm_gx5_45
* functional version of ROS node, but no publishing yet
* adding ROS node - compiles
* Update README.md
* Merge branch 'master' of github.com:bsb808/microstrain_3dm_gx5_45
* working version
* Update README.md
* compilable version with user-devined port string
* reorg
* Merge branch 'master' of github.com:bsb808/microstrain_3dm_gx5_45
* working version, but all in C.  To use the serial library will need to convert to C++
* Update README.md
* Create README.md
* incuding the MIP SDK files
* working version that sorces the MIP SDK
* Initial commit
* Contributors: Administrator, Bingham, Brian S, Brian Bingham, FRL, Field Robotics Lab, Jeff Schmidt, L. James Azzalini, Shreya Subbu, Shreya Subramaniam, Tony Baltovski, Wolfgang Merkt, ljazzal, pvechersky
