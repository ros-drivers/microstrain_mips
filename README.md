## Description

Interface (driver) software, including ROS node, for inertial sensors compatible with the [Microstrain Communication Library (MSCL)](https://github.com/LORD-MicroStrain/MSCL).

MSCL is developed by [LORD Sensing - Microstrain](http://microstrain.com) in Williston, VT. 


## Build Instructions

#### MSCL
MSCL pre-built packages can be found here: [MSCL Packages](https://github.com/LORD-MicroStrain/MSCL/releases/tag/v62.0.0)

We do our best to keep ROS-MSCL up-to-date with the latest MSCL changes, but sometimes there is a delay. The currently supported version of MSCL is [v62.0.0](https://github.com/LORD-MicroStrain/MSCL/releases/tag/v62.0.0)

Install instructions can be found here: [How to Use MSCL](https://github.com/LORD-MicroStrain/MSCL/blob/master/HowToUseMSCL.md#linux)

If you choose to install MSCL at a location other than /usr/share, [CMakeLists.txt](https://github.com/LORD-MicroStrain/ROS-MSCL/blob/master/ros_mscl/CMakeLists.txt) will need to be updated with the install path.

#### Building from source
1. Install ROS and create a workspace: [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

2. Move the entire ROS-MSCL folder (ros_mscl and mscl_msgs for just source) to the your_workspace/src directory.

3. Locate and register the ros_mscl package: `rospack find ros_mscl`

4. Build your workspace:
        
        cd ~/your_workspace
        catkin_make
        source ~/your_workspace/devel/setup.bash
   The source command may need to be run in each terminal prior to launching a ROS node.
#### Launch the node and publish data
The following command will launch the driver. Keep in mind each instance needs to be run in a separate terminal.
            
        roslaunch ros_mscl microstrain.launch
Optional launch parameters:
- name: namespace the node will publish messages to, default: gx5
- port: serial port name to connect to the device over, default: /dev/ttyACM0
- baudrate: baud rate to open the connection with, default: 115200
- imu_rate: sample rate for IMU data (hz), default: 100
- debug: output debug info? default: false
- diagnostics: output diagnostic info? default: true
    
To check published topics:
        
    rostopic list

**Example**: Connect to and publish data from two devices simultaneously  
In two different terminals:
    
    roslaunch ros_mscl microstrain.launch name:=sensor1234

    roslaunch ros_mscl microstrain.launch name:=bestSensor port:=/dev/ttyACM1
This will launch two nodes that publish data to different namespaces:
- sensor1234, connected over port: /dev/ttyACM0
- bestSensor, connected over port: /dev/ttyACM1

An example subscriber node can be found here: [ROS-MSCL Examples](https://github.com/LORD-MicroStrain/ROS-MSCL/tree/master/Examples)  


## Docker Integration

### VSCode

The easiest way to use docker while still using an IDE is to use VSCode as an IDE. Follow the steps below to develop on this repo in a docker container

1. Install the following dependencies:
    1. [VSCode](https://code.visualstudio.com/)
    1. [Docker](https://docs.docker.com/get-docker/)
1. Open VSCode and install the following [plugins](https://code.visualstudio.com/docs/editor/extension-marketplace):
    1. [VSCode Docker plugin](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker)
    1. [VSCode Remote Containers plugin](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
1. Open this directory in a container by following [this guide](https://code.visualstudio.com/docs/remote/containers#_quick-start-open-an-existing-folder-in-a-container)
    1. Due to a bug in the remote container plugin, you will need to refresh the window once it comes up. To do this, type `Ctrl+Shift+p` and type `Reload Window` and hit enter. Note that this will have to be repeated every time the container is rebuilt
1. Once the folder is open in VSCode, you can build the project by running `Ctrl+Shift+B` to trigger a build, or `Ctrl+p` to open quick open, then type `task build` and hit enter
1. You can run the project by following [this guide](https://code.visualstudio.com/docs/editor/debugging)

### Make

If you are comfortable working from the command line, or want to produce runtime images, the [Makefile](./devcontainer/Makefile) in the [.devcontainer](./devcontainer) directory
can be used to build docker images, run a shell inside the docker images and produce a runtime image. Follow the steps below to setup your environment to use the `Makefile`

1. Install the following dependencies:
    1. [Make](https://www.gnu.org/software/make/)
    1. [Docker](https://docs.docker.com/get-docker/)

The `Makefile` exposes the following tasks. They can all be run from the `.devcontainer` directory:
* `make build-shell` - Builds the docker image and starts a shell session in the image allowing the user to develop and build the ROS project using common commands such as `catkin_make`
* `make image` - Builds the runtim image that contains only the required dependencies and the ROS node. The resulting image is names `ros-mscl`
* `make clean` - Cleans up after the above two tasks

## License
ROS-MSCL is released under the MIT License - see the `LICENSE` file in the source distribution.

Copyright (c)  2021, Parker Hannifin Corp.

