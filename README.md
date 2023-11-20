# kortex_hardware
![build](https://github.com/empriselab/kortex_hardware/actions/workflows/build-test.yml/badge.svg)

This repository provides a `ros_control` (ROS1) hardware interface for the Kinova [Gen3](https://www.kinovarobotics.com/product/gen3-robots) robot.

## Main Features

This hardware interface provides the following features:
* Read/write capabilities for joint positions, velocities, and effort for the Gen3 arm. For position and velocity control, we currently support high-level servoing mode, which runs at 40Hz. We provide support for effort (torque/control) control using the low-level servoing mode of the robot, which runs at 1KHz.
* Read/write capabilities for the Robotiq-2f-85 gripper using Kortex API. The servoing level depends on the servoing mode for the arm.
* ROS service for switching between different control modes. (This can also be accomplished using joint mode controllers)
* Software E-Stop mode that puts the robot in effort mode and performs gravity compensation, allowing users to easily move the arm to the desired position.

## Requirements

Following are the external API and library requirements for using this hardware interface.
* Kortex API (read robot state and send commands to the robot)
* Pinocchio (for gravity compensation when using effort control)
* kortex_description (ROS package containing Gen3 URDF and xacro files, also required for gravity compensation)
* Boost and Eigen (base dependencies)

## Installation

Assuming ROS1 is already installed, we additionally require Pinocchio for gravity compensation at the hardware interface level. Pinocchio officially recommends installing the binaries via the ROS PPA when using it with ROS packages using: 

```sudo apt install ros-$ROS_DISTRO-pinocchio```

**Note**: Some users encounter Eigen-related errors using this error. For such cases, we additionally suggest using their [official documentation](https://stack-of-tasks.github.io/pinocchio/download.html).

If you are interested in testing the interface with ROS controllers, you also need to run `sudo apt install ros-$ROS_DISTRO-ros-controllers`.

## Usage

One can use this package as the hardware interface package for the Gen3 robot provided the appropriate URDF files. 
We also provide testing instructions to allow users to quickly try out this interface with `ros_control`.

Following are the instructions for the initial setup:
```
# create catkin workspace
mkdir -p ~/hw_test_ws/src
cd ~/hw_test_ws/src

# clone and build the required packages
git clone https://github.com/empriselab/kortex_hardware
git clone https://github.com/empriselab/kortex_description
cd .. && catkin build

```

Before you start using the hardware interface, make sure to have the connection details, including the IP address for your robot. It is preferred to have the robot and the system running ROS on the same network and connected via ethernet (especially important for torque control). For safety purposes, **make sure to have the e-stop next to you when testing.**

Running instructions for testing the hardware interface:
```
# In terminal 1
cd ~/hw_test_ws && source devel/setup.bash
roslaunch kortex_hardware gen3_hardware.launch ip_address:=<robot-ip-address> dof:=<robot-dof> # see launch file for more parameters
# Robot IP should be the same as the one you use to access the web app.

# In terminal 2
cd ~/hw_test_ws && source devel/setup.bash
rosrun kortex_hardware test_modes.py velocity # <position/velocity/effort/stop>

Command: <enter joint value> # in radians
DOF: <specify joint number between 1-NDOF>
# Ctrl + \ to exit
```

**Note**: when switching to position mode for the first time, the robot moves to the candlestick position.

In case you are not able to connect to the robot or are unable to command it, it is always handy to use the Kinova Web App to check for connection and clear any faults.

## Contribution
Any contributions related to bug fixes/feature additions/documentation are welcome! Contributing is simple. Just follow the following steps:
* Fork the repository
* Create a new branch with an appropriate name
* Make changes, test, and commit
* Format the code using `catkin build kortex_hardware --no-deps --make-args format`
* Send a PR to the main repository and tag the maintainers to request a review.

