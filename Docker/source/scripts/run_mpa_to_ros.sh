#!/bin/bash
#
# source ros env
source ros_environment.sh

# set LD_LIBRARY_PATH to first check for modalai libs @ /usr/lib64 & /usr/lib
export LD_LIBRARY_PATH=/usr/lib64:/usr/lib:${LD_LIBRARY_PATH}

# launch mpa_to_ros_node
roslaunch voxl_mpa_to_ros voxl_mpa_to_ros.launch
