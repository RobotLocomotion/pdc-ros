#!/bin/bash

set -euxo pipefail

apt-get update

sudo pip install requests
sudo pip install matplotlib
sudo pip install scipy
sudo pip install imageio

sudo pip install scikit-image
sudo pip install tensorboard

sudo pip install sklearn

sudo pip install opencv-contrib-python

sudo pip install tensorboard_logger \
    tensorflow

# Install ROS Kinetic
apt install -y --no-install-recommends lsb-release
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

apt-get update

apt install python-tk

#apt install -y --no-install-recommends \
#  ros-kinetic-desktop \
  #ros-kinetic-librealsense \
  #ros-kinetic-realsense-camera \
  #ros-kinetic-moveit \
#  ros-kinetic-openni2-launch \
#  ros-kinetic-image-pipeline
