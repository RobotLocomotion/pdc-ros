#!/bin/bash
set -e

export PDC_ROS_SOURCE_DIR=~/code
export DATA_DIR=~/data
export DC_DATA_DIR=$DATA_DIR/pdc
export DC_SOURCE_DIR=$PDC_ROS_SOURCE_DIR/pytorch-dense-correspondence
export PDC_ROS_BUILD_DIR=$PDC_ROS_SOURCE_DIR/build
export PDC_ROS_SANDBOX_DIR=~/sandbox

# poser
export POSER_BUILD_DIR=$PDC_ROS_BUILD_DIR/poser
export POSER_OUTPUT_DIR=$PDC_ROS_SANDBOX_DIR/poser

# mankey
export MANKEY_OUTPUT_DIR=$PDC_ROS_SANDBOX_DIR/mankey

# location of custom COCO data
export COCO_CUSTOM_DATA_DIR=$DC_DATA_DIR/coco

# sandbox location, Detectron pre-trained weights will end up here
export SANDBOX_DIR=$DC_SOURCE_DIR/sandbox


source $DC_SOURCE_DIR/config/setup_environment.sh
source $PDC_ROS_SOURCE_DIR/config/setup_environment.sh

exec "$@"

cd $PDC_ROS_SOURCE_DIR
