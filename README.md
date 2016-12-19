# microstrain_3dm_gx5_45

## Build instructions.  

The concept is to build this drive along with the Microstrain SDK (Version 1.1).  Currently the repo contains the SDK files,
but in the future, it might make sense to not include them - or to substitute a newer version.  
Here are the steps I took to build the code...
 * Dowload the zip archive "MIP C Code Sample for Windows and Linux Version 1.1" from the Microstrain website
 * Extract the archive into the directory "MIPSDK".  The first directory in MIPSDK should be "C"
 * In C/Library/User Functions/mip_sdk_user_functions.c, comment out the line "#include <windows.h>
 * cd catkin_sw
 * catkin_make
 
 
