#!/bin/bash

set -e

cd catkin_ws

source /opt/ros/kinetic/setup.bash

catkin_make

catkin_make install
