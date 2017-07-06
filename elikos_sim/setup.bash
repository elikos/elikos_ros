#!/bin/bash
GAZEBO_SIM_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"/src/gazebo_sim

source $GAZEBO_SIM_DIR/Firmware/Tools/setup_gazebo.bash $GAZEBO_SIM_DIR/Firmware $GAZEBO_SIM_DIR/Firmware/build_posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$GAZEBO_SIM_DIR/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$GAZEBO_SIM_DIR/Firmware/Tools/sitl_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$GAZEBO_SIM_DIR/elikos_gazebo_models/models

