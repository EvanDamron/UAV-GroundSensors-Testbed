# Raspberry Pi Data Collection and Transfer

## Overview
This repository contains code for collecting data from an LTR390 UV sensor, storing it, and transferring it to a UAV. The project consists of scripts for reading and storing sensor data, as well as managing WiFi connections and data transfers to the drone.

## Repository Structure

```plaintext
RaspberryPi/
├── uav-data-collection/
│   ├── data-collection/
│   │   ├── read_data.py
│   │   ├── store_data.py
│   ├── data-transfer/
│   │   ├── data_transfer.py
│   │   ├── wifi_connection.py
├── docs/
│   ├── RPI_README.md
```

## Scripts

### 1. `data-collection/read_data.py`
This script reads data from the LTR390 UV sensor and prints it to the terminal.

#### Usage
Run the script with the following command:
`python3 read_data.py`

### 2. `data-collection/store_data.py`
This script reads data from the LTR390 UV sensor and stores it in a CSV file with timestamps every 5 seconds.

#### Usage
For this script to work, the I2C interface must be enabled on the Raspberry Pi. In our experiments, this script was run as a service so that data collection would begin as soon as the Pi was connected to power.

### 3. `data-transfer/data_transfer.py`
This script handles the automatic connection to the drone and transfers the collected data. It attempts to connect to the drone's WiFi network, send the data file, and handle any communication required during the transfer.

### Usage
Run the script with the following command:
`python3 data_transfer.py`

Make sure to configure the WiFi SSID, password, IP address, and data file path in the script before running it.


### 4. `data-collection/wifi_connection.py`
This script provides helper functions for data_transfer.py to manage WiFi connections. It can reset the WiFi interface, connect to a specified network, and acquire an IP address. When run as the main file, it prompts the user to connect to either their home network or the drone.
#### Usage
Run the script with the following command to use it interactively.
`python3 wifi_connection.py`
