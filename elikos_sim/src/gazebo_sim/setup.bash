#!/bin/bash
source Firmware/Tools/setup_gazebo.bash $(pwd)/Firmware $(pwd)/Firmware/build_posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Firmware/Tools/sitl_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/elikos_gazebo_models/models

