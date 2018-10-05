#!/bin/bash

set -euxo pipefail

apt-get update

apt-get install -y \
  ros-kinetic-desktop
  #ros-kinetic-librealsense \
  #ros-kinetic-realsense-camera \
  #ros-kinetic-moveit \

apt-get install -y  \
  ros-kinetic-openni2-launch \
  ros-kinetic-image-pipeline
