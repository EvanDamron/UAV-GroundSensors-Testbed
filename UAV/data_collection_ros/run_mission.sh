#!/bin/bash
#
# Start manager, flight_execution, and communication nodes which runs the mission and saves the output to a log file.
# It takes the name of the sensor configuration file located in sensor_configurations as an argument.

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <sensor_config_file_name>"
    exit 1
fi

sensor_config_file_name=$1
sensor_config_file="/root/yoctohome/data_collection_ros/sensor_configurations/${sensor_config_file_name}"

source ros_environment.sh
timestamp=$(date +%H%M%S)
log_file="./logs/log-${timestamp}.log"
stdbuf -oL -eL roslaunch drone_flight mission.launch sensor_config_file:=${sensor_config_file} 2>&1 | tee "${log_file}"

