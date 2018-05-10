#!/bin/bash
set -e

function use_pytorch_dense_correspondence()
{
    source ~/code/config/setup_environment.sh
    echo "using pdc"
}

export -f use_pytorch_dense_correspondence

function use_ros()
{	
	source /opt/ros/kinetic/setup.bash
    source ~/catkin_ws/devel/setup.bash
    echo "using ros"
}

export -f use_ros

function use_all()
{	
	use_ros
    use_pytorch_dense_correspondence
    echo "using pdc and ros"
}

export -f use_all

exec "$@"

cd ~/code
