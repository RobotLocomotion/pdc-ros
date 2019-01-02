#!/bin/bash

set -euxo pipefail

# Install ROS Kinetic
apt install -y --no-install-recommends lsb-release
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

apt-get update

apt install -y --no-install-recommends \
  ros-kinetic-desktop \

apt-get install -y  \
  ros-kinetic-openni2-launch \
  ros-kinetic-image-pipeline \
  ros-kinetic-ros-numpy
