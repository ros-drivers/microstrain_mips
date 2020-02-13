## Description

Interface (driver) software, including ROS node, for inertial sensors compatible with the [Microstrain Communication Library (MSCL)] (https://github.com/LORD-MicroStrain/MSCL).

MSCL is developed by [LORD Sensing - Microstrain](http://microstrain.com) in Williston, VT. 


## Build Instructions

Download and install MSCL at /usr/share

#### Building from source

    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash

##### Pre-built MSCL Binaries/Packages (v52.2.1)
Windows:
[C++](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/mscl_52.2.1_Windows_C++.zip) | 
[Python 2.7](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/mscl_52.2.1_Windows_Python2.7.zip) |
[Python 3.6](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/mscl_52.2.1_Windows_Python3.6.zip) |
[.NET](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/mscl_52.2.1_Windows_DotNet.zip)

Debian:
  * x64:
  [C++](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl_52.2.1_amd64.deb) |
  [Python 2](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/python2-mscl_52.2.1_amd64.deb) |
  [Python 3](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/python3-mscl_52.2.1_amd64.deb)
  * arm64:
  [C++](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl_52.2.1_arm64.deb) |
  [Python 2](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/python2-mscl_52.2.1_arm64.deb) |
  [Python 3](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/python3-mscl_52.2.1_arm64.deb)
  * armhf (Raspbian):
  [C++](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl_52.2.1_armhf.deb) |
  [Python 2](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/python2-mscl_52.2.1_armhf.deb) |
  [Python 3](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/python3-mscl_52.2.1_armhf.deb)

RPM:
  * x64:
  [C++](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl-52.2.1_x86_64.rpm) |
  [Python 2](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/python2-mscl-52.2.1_x86_64.rpm) |
  [Python 3](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/python3-mscl-52.2.1_x86_64.rpm)
  * arm64:
  [C++](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl-52.2.1_aarch64.rpm) |
  [Python 2](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/python2-mscl-52.2.1_aarch64.rpm) |
  [Python 3](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/python3-mscl-52.2.1_aarch64.rpm)
  * CentOS:
  [C++](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/c++-mscl-52.2.1_x86_64_centos7.6.1810.rpm) |
  [Python 2](https://github.com/LORD-MicroStrain/MSCL/releases/download/v52.2.1/python2-mscl-52.2.1_x86_64_centos7.6.1810.rpm)

If the pre-built packages aren't available on your platform, you can build MSCL from source. This library will search for MSCL at /usr/share.

Instructions can be found here:
[Building MSCL on Windows](https://github.com/LORD-MicroStrain/MSCL/blob/master/BuildScripts/buildReadme_Windows.md) | 
[Building MSCL on Linux](https://github.com/LORD-MicroStrain/MSCL/blob/master/BuildScripts/buildReadme_Linux.md)

### Documentation

[How to use MSCL](https://github.com/LORD-MicroStrain/MSCL/blob/master/HowToUseMSCL.md)
  
[FAQs](https://github.com/LORD-MicroStrain/MSCL/blob/master/FAQs.md)


## License
ROS-MSCL is released under the MIT License - see the `LICENSE` file in the source distribution.

Copyright (c)  2020, Parker Hannifin Corp.
