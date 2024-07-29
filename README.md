# UAV-GroundSensors-Testbed

## Overview
This project demonstrates an autonomous UAV that visits ground sensors, establishes a wireless connection, and collects data. The UAV is programmed using MAVROS, and the ground sensors are Raspberry Pis.

## Features
- Autonomous navigation of the ModalAI M500 UAV using MAVROS
- Wireless data transmission from RaspberryPi to UAV via the UAV's WiFi hotspot
- UV data collection by Raspberry Pis

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
│   ├── src/
│   │   ├── main_sensor_script.py
│   │   ├── other_sensor_scripts.py
│   ├── config/
│   │   ├── sensor_config.yaml
│   ├── docs/
│   │   ├── Pi_README.md
│   ├── tests/
│   │   ├── test_sensor_script.py
│   └── requirements.txt
├── scripts/
│   ├── setup_uav.sh
│   ├── setup_raspberry_pi.sh
├── images/
│   ├── uav_setup.png
│   ├── raspberry_pi_setup.png
├── README.md
├── LICENSE
└── .gitignore
