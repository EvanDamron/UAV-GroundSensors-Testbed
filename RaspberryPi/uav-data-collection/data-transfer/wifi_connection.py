#!/usr/bin/env python3

import subprocess
import time
import tempfile


def reset_wifi_interface(interface):
    """Reset the WiFi interface."""
    print(f"Resetting WiFi interface {interface}...")
    subprocess.run(["sudo", "ifconfig", interface, "down"], check=True)
    try:
        subprocess.run(["sudo", "pkill", "wpa_supplicant"], check=True)
    except:
        """wpa_supplicant was still running"""
    subprocess.run(["sudo", "ifconfig", interface, "up"], check=True)


def connect_to_network(interface, ssid, password):
    """Connect to a WiFi network."""
    # Create a temporary file for the wpa_supplicant configuration
    with tempfile.NamedTemporaryFile('w', delete=False) as wpa_conf:
        wpa_conf.write(f"""
country=US
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={{
    ssid="{ssid}"
    psk="{password}"
    key_mgmt=WPA-PSK
}}
""")
        wpa_conf_path = wpa_conf.name

    # Start wpa_supplicant with the temporary configuration file
    subprocess.run(["sudo", "wpa_supplicant", "-B", "-i", interface, "-c", wpa_conf_path], check=True)
    # Remove the temporary configuration file
    subprocess.run(["rm", wpa_conf_path], check=True)


def acquire_ip(interface):
    """Acquire an IP address using dhcpcd"""
    subprocess.run(["sudo", "dhcpcd", interface], check=True)


def check_connection(interface, ssid):
    """Check if the WiFi connection is successful."""
    time.sleep(7)     # Wait for the connection to establish
    result = subprocess.run(["iwconfig", interface], capture_output=True, text=True)
    print(result.stdout)
    # Check if the SSID is in the output of iwconfig
    if ssid in result.stdout:
        print(f"Successfully connected to {ssid}")
    else:
        print(f"Failed to connect to {ssid}")

def scan_for_ssid(interface, ssid):
    """Scan for a specific SSID"""
    result = subprocess.run(["sudo", "iwlist", interface, "scan"], capture_output=True, text=True)
    if f'ESSID:"{ssid}"' in result.stdout:
        return True
    return False

def connect_to_wifi(ssid, password):
    """Main function to connect to a WiFi network."""
    try:
        reset_wifi_interface('wlan0')

        if scan_for_ssid('wlan0', ssid):
            print(f"Network {ssid} found. Connecting...")
            connect_to_network('wlan0', ssid, password)
            acquire_ip('wlan0')
            check_connection('wlan0', ssid)
        else:
            time.sleep(7)
            print(f"Network {ssid} not found.")
    except subprocess.TimeoutExpired:
        print("WiFi connection script timed out")
    except subprocess.CalledProcessError as e:
        print(f"Failed to execute WiFi connection script: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    # Prompt the user to choose between connecting to the drone or the router
    choice = input("Do you want to connect to the drone or the router? (drone/router): ").strip().lower()

    if choice == "drone":
        connect_to_wifi("INSERT DRONE SSID", "INSERT DRONE PASSWORD")
    elif choice == "router":
        connect_to_wifi("INSERT ROUTER SSID", "INSERT ROUTER PASSWORD")
    else:
        print("Invalid choice. Please enter 'drone' or 'router'.")
