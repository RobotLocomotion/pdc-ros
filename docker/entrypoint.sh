#!/bin/bash
set -e

function use_pytorch_dense_correspondence()
{
    source ~/code/config/setup_environment.sh
}

export -f use_pytorch_dense_correspondence

function use_ros()
{	
	source /opt/ros/kinetic/setup.bash
    source ~/catkin_ws/devel/setup.bash
}

export -f use_ros

function use_all()
{	
	use_ros
    use_pytorch_dense_correspondence
}

export -f use_all

exec "$@"

cd ~/code
