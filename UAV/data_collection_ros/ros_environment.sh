#!/bin/bash
#RUNS INSIDE KINETIC DOCKER CONTAINER

# load main ros environment
source /opt/ros/kinetic/setup.bash

source catkin_ws/install/setup.bash
source catkin_ws/devel/setup.bash
unset ROS_HOSTNAME

# configure ROS IPs here
export ROS_MASTER_IP=127.0.0.1
export ROS_IP=192.168.8.1
export ROS_MASTER_URI=http://${ROS_MASTER_IP}:11311/
# mavros needs to know what PX4's system id is
export PX4_SYS_ID=1

