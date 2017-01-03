#!/bin/bash

source Firmware/Tools/setup_gazebo.bash $(pwd) $(pwd)/Firmware/build_posix_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Firmware/Tools/sitl_gazebo

