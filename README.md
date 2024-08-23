# UAV-GroundSensors-Testbed

## Overview
This project demonstrates an autonomous UAV that visits ground sensors, establishes a wireless connection, and collects data. The UAV is programmed using MAVROS, and the ground sensors are Raspberry Pis. For more details on the two aspects of the project, see UAV/docs/UAV_README.md and RaspberryPi/docs/RPI_README.md.

## Features
- Autonomous navigation of the ModalAI M500 UAV using MAVROS
- Wireless data transmission from RaspberryPi to UAV via the UAV's WiFi hotspot
- Ultra-violet (UV) data collection by Raspberry Pis

## Repository Structure
```plaintext
UAV-GroundSensors-Testbed/
├── UAV/
│   ├── data_collection_ros/
│   │   ├── catkin_ws/
│   │   │   ├── src/
│   │   │   │   ├── drone_comm/
│   │   │   │   │   ├── scripts/
│   │   │   │   │      ├── communication.py
│   │   │   │   │   ├── CMakeLists.txt
│   │   │   │   │   ├── package.xml
│   │   │   │   ├── drone_flight/
│   │   │   │   │   ├── src/
|   |   |   |   |   |   ├── flight_execution.cpp
|   |   |   |   |   |   ├── manager.cpp
|   |   |   |   |   ├── launch/
|   |   |   |   |   |   ├── mission.launch
│   │   │   │   │   ├── CMakeLists.txt
│   │   │   │   │   ├── package.xml
|   |   ├── logs/
|   |   ├── sensor_configurations/
|   |   |   ├── sample_config.txt
|   |   ├── build.sh
|   |   ├── clean.sh
|   |   ├── ros_environment.sh
|   |   ├── run_mavros.sh
|   |   ├── run_mission.sh
│   ├── docs/
│   │   ├── UAV_README.md
├── RaspberryPi/
│   ├── uav-data-collection/
│   │   ├── data-collection/
|   |   |   ├── read_data.py
|   |   |   ├── store_data.py
│   │   ├── data-transfer/
|   |   |   ├── data_transfer.py
|   |   |   ├── wifi_connection.py
│   ├── docs/
│   │   ├── RPI_README.md
└── README.md
