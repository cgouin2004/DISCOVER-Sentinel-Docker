#!/bin/bash
#

# load main ros environment
if [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
elif [ -f /opt/ros/kinetic/setup.bash ]; then
    source /opt/ros/kinetic/setup.bash
elif [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
fi

if [ -f ~/ros_ws/install/setup.bash ]; then
    source ~/ros_ws/install/setup.bash
fi

unset ROS_HOSTNAME

# configure ROS IPs here
export ROS_MASTER_IP=127.0.0.1
export ROS_IP=192.168.8.1
export ROS_MASTER_URI=http://${ROS_MASTER_IP}:11311/
# mavros needs to know what PX4's system id is
export PX4_SYS_ID=1

