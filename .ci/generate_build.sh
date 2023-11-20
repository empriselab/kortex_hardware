#!/bin/bash
set -ex

# setup ros environment in Docker Container
source "/opt/ros/$ROS_DISTRO/setup.bash" --
cd /
# create workspace directory where package will be built
mkdir workspace
cd workspace
cp -r ${GITHUB_WORKSPACE}/. src/
# Update and install packages
apt-get update
apt-get -y install python3-pip
# Install catkin build
pip3 install -U catkin_tools
# Install kortex_hardware package deps
apt-get install python3-rosdep
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y
# Install format checker
apt-get install clang-format-10 -y
apt-get update
#build
catkin build 