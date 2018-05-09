#!/bin/bash
#
# This script is run by the dockerfile during the docker build.
#

set -exu

root_dir=$(pwd)
install_dir=$root_dir/install

apt-get update

# convenience programs to have inside the docker
apt install --no-install-recommends \
  wget \
  libglib2.0-dev \
  libqt4-dev \
  libqt4-opengl \
  libx11-dev \
  libxext-dev \
  libxt-dev \
  mesa-utils \
  libglu1-mesa-dev \
  python-dev \
  python-lxml \
  python-numpy \
  python-scipy \
  python-yaml
