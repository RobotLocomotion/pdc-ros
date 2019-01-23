#!/bin/bash
set -e

export PDC_ROS_SOURCE_DIR=~/code
export DATA_DIR=~/data
export DC_DATA_DIR=$DATA_DIR/pdc
export DC_SOURCE_DIR=$PDC_ROS_SOURCE_DIR/pytorch-dense-correspondence
export PDC_ROS_BUILD_DIR=$PDC_ROS_SOURCE_DIR/build
export POSER_BUILD_DIR=$PDC_ROS_BUILD_DIR/poser

export PDC_ROS_SANDBOX_DIR=~/sandbox
export POSER_OUTPUT_DIR=$PDC_ROS_SANDBOX_DIR/poser

# location of custom COCO data
export COCO_CUSTOM_DATA_DIR=$DC_DATA_DIR/coco

# sandbox location, Detectron pre-trained weights will end up here
export SANDBOX_DIR=$DC_SOURCE_DIR/sandbox

function use_pytorch_dense_correspondence()
{
    source $DC_SOURCE_DIR/config/setup_environment.sh
    echo "using pdc"
}

export -f use_pytorch_dense_correspondence

function use_director()
{
    export PATH=$PATH:~/director/bin
}
export -f use_director


function use_ros()
{	
	source /opt/ros/kinetic/setup.bash
    echo "using ros"
}

export -f use_ros

function use_pdc_ros()
{
    source $PDC_ROS_SOURCE_DIR/catkin_ws/devel/setup.bash
    use_pytorch_dense_correspondence
    echo "using pdc_ros"
}

export -f use_pdc_ros

function use_all()
{	
	use_ros
    use_pdc_ros
    echo "using pdc and ros"
}


use_drake(){
    export PYTHONPATH=/opt/drake/lib/python2.7/site-packages:${PYTHONPATH}
}

export -f use_drake

export -f use_all

exec "$@"

cd $PDC_ROS_SOURCE_DIR
