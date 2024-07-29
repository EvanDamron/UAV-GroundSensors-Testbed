===#!/usr/bin/env python3

import logging
import subprocess
import socket
import time
import os
from datetime import datetime
from wifi_connection import connect_to_wifi, reset_wifi_interface
from threading import Thread

# Configure logging with timestamp in filename
now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_filename = f"voxl_data_transfer_{now}.log"
log_dir = os.path.join(os.getcwd(), 'logs')
os.makedirs(log_dir, exist_ok=True)
logging.basicConfig(filename=os.path.join(log_dir, log_filename), level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s')

def is_connected_to_ssid(ssid):
    """
    Check if the device is connected to a specific SSID.
    """
    time.sleep(3)
    try:
        result = subprocess.check_output(['iwgetid', '-r']).strip().decode('utf-8')
        return result == ssid
    except subprocess.CalledProcessError:
        return False

def send_file_to_drone(path, sock):
    """
    Send a file to the drone through the specified socket.
    """
    try:
        with open(path, 'rb') as f:
            chunk = f.read(1024)
            while chunk:
                sock.sendall(chunk)
                chunk = f.read(1024)
        logging.info("Data file sent successfully")
        print("Data file sent successfully")
    except Exception as e:
        logging.error(f"Failed to send message: {e}")
        print(f"Failed to send message: {e}")

def send_file_with_timeout(path, sock, timeout=4):
    """
    Send a file to the drone with a timeout. If the sending takes longer than the timeout, it stops.
    """
    thread = Thread(target=send_file_to_drone, args=(path, sock))
    thread.start()
    thread.join(timeout)
    if thread.is_alive():
        logging.error("Timeout reached while sending file")
        print("Timeout reached while sending file")
        return False
    return True

def receive_message(sock):
    """
    Receive a message from the drone through the specified socket.
    """
    try:
        message = sock.recv(1024).decode('utf-8')
        logging.info(f"Received message from drone: {message}")
        return message
    except Exception as e:
        logging.error(f"Failed to receive message: {e}")
        return None


# Configuration parameters
DRONE_SSID = "INSERT SSID HERE"
DRONE_PASSWORD ="INSERT PASSWORD HERE"
DRONE_IP = "INSERT IP ADDRESS HERE"
DRONE_PORT = 12345
DATA_PATH = os.path.join('..', 'data', 'INSERT DATA FILE HERE')


while True:
    # Connect to drone and send file, retrying until it is succesfully sent
    if is_connected_to_ssid(DRONE_SSID):
        print(f"connected to SSID {DRONE_SSID}")
        logging.info("RPI is connected to drone's wifi hotspot.")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect((DRONE_IP, DRONE_PORT))
        except:
            logging.info("Socket couldn't connect. Trying again in 2 seconds")
            time.sleep(2)
        message = receive_message(sock)
        if message == "go to sleep":
            print("Received 'go to sleep' message. Disconnecting...")
            logging.info("Received 'go to sleep' message. Disconnecting...")
            sock.close()
            reset_wifi_interface('wlan0')
            time.sleep(10)
        elif message == "ready to receive":
            print("Received 'ready to receive' message. Transmitting...")
            logging.info("Received 'ready to receive' message. Transmitting...")
            ack_received = False
            retries = 3
            for attempt in range(retries):
                if send_file_with_timeout(DATA_PATH, sock):
                    sock.shutdown(socket.SHUT_WR)
                    ack = receive_message(sock)
                    if ack == "ack":
                        logging.info("Acknowledgment received from drone.")
                        print("Received Ack from drone")
                        ack_received = True
                        break
                    else:
                        print("No ack received, retrying...")
                        logging.info("No acknowledgment received. Retrying...")
                else:
                    print("File transmission timeout. Retrying")
                    logging.info("File transmission timeout. Retrying...")

            if not ack_received:
                logging.error("Failed to receive acknowledgment after multiple attempts. Restarting loop.")
                sock.close()
                continue

            sock.close()
            reset_wifi_interface('wlan0')
            break
        else:
            print("Haven't received any message. Checking again in 2 seconds.")
            logging.info("Haven't received any message. Checking again in 2 seconds.")
            time.sleep(2)
    else:
        print(f"not connected to SSID {DRONE_SSID}")
        logging.info(f"not connected to SSID {DRONE_SSID}, attempting to connect every 4 seconds.")
        connect_to_wifi(DRONE_SSID, DRONE_PASSWORD)
        time.sleep(4)
