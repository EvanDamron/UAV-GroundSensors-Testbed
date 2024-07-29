# UAV README
## Overview
This repository contains the code and configuration files for the UAV side of UAV-GroundSensors-Testbed project. The UAV autonomously visits ground sensors, collects data, and returns to base. This README provides detailed information about the scripts, configuration files, and ROS nodes on the drone. The code is meant to be built and run on the ModalAI M500 within the ROS Kinetic Docker Container, which can be found at https://developer.modalai.com/.

## Repository Structure
```plaintext
UAV-GroundSensors-Testbed/
├── UAV/
│   ├── data_collection_ros/
│   │   ├── catkin_ws/
│   │   │   ├── src/
│   │   │   │   ├── drone_comm/
│   │   │   │   │   ├── scripts/
│   │   │   │   │   │   ├── communication.py
│   │   │   │   │   ├── CMakeLists.txt
│   │   │   │   │   ├── package.xml
│   │   │   │   ├── drone_flight/
│   │   │   │   │   ├── src/
│   │   │   │   │   │   ├── flight_execution.cpp
│   │   │   │   │   │   ├── manager.cpp
│   │   │   │   │   ├── launch/
│   │   │   │   │   │   ├── mission.launch
│   │   │   │   │   ├── CMakeLists.txt
│   │   │   │   │   ├── package.xml
│   │   ├── logs/
│   │   ├── sensor_configurations/
│   │   │   ├── sample_config.txt
│   │   ├── build.sh
│   │   ├── clean.sh
│   │   ├── ros_environment.sh
│   │   ├── run_mavros.sh
│   │   ├── run_mission.sh
│   ├── docs/
│   │   ├── UAV_README.md

## ROS Nodes

### 1. `drone_comm/scripts/communication.py`
This script handles the communication between the UAV and the ground sensors. It sets up a TCP server to receive data from the sensors and save it to the specified location.

### 2. `drone_flight/src/flight_execution.cpp`
This node sends all necessary MAVLink messages to the drone. It manages the drone's flight, including takeoff, waypoint traversal, and landing. It listens for waypoints from the `waypoint_manager` node.

### 3. `drone_flight/src/manager.cpp`
This node stores the locations of the waypoints and publishes them to `flight_execution.cpp`. It also manages the IP address and save location for each sensor and ensures the data is collected from each sensor.

## Scripts and Configuration Files

### 1. `build.sh`
A script to build the ROS workspace.

### 2. `clean.sh`
A script to clean the build artifacts from the ROS workspace.

### 3. `ros_environment.sh`
A script to set up the ROS environment variables.

### 4. `run_mavros.sh`
A script to launch MAVROS.

### 5. `run_mission.sh`
A script to run the mission, which includes launching necessary ROS nodes and managing the UAV's flight.

### 6. `sensor_configurations/sample_config.txt`
A sample configuration file for sensors, including their locations and IP addresses.

