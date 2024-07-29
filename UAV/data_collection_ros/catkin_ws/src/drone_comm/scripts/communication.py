#!/usr/bin/env python

import rospy
import socket
import signal
import sys
from std_msgs.msg import String
import threading
import select

sock = None
done_pub = None
DRONE_IP = '0.0.0.0'
DRONE_PORT = 12345
current_ip = None
current_save_location = None

def signal_handler(sig, frame):
    """
    Signal handler for graceful shutdown.
    Closes the socket and exits the program when a termination signal is received.
    """
    global sock
    rospy.loginfo('[communication] Signal handler called with signal %s', sig)
    if sock:
	sock.close()
	rospy.loginfo('[communication] Socket closed')
    sys.exit(0)

def ensure_socket_closed(drone_ip, drone_port):
    """
    Ensures the socket is closed before starting the server.
    Attempts to bind and immediately close the socket to free up the port.
    """
    try:
        # Attempt to bind and then immediately close the socket
        temp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        temp_sock.bind((drone_ip, drone_port))
        temp_sock.close()
        rospy.loginfo('[communication] Ensured socket on port %d is closed' % drone_port)
    except socket.error as e:
        rospy.logwarn('[communication] Socket on port %d might already be in use: %s' % (drone_port, e))

def start_server(drone_ip, drone_port):
    """
    Starts the server to listen for incoming connections from sensors.
    Receives data from the sensor and saves it to the specified location.
    Publishes a 'done' message when data transfer is complete.
    """
    global sock
    global current_ip
    global current_save_location
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Allow reusing address
        server_address = (drone_ip, drone_port)
        rospy.loginfo('[communication] Starting server on %s port %d' % (drone_ip, drone_port))
        sock.bind(server_address)
        sock.listen(1)
	
        while not rospy.is_shutdown():
	    # Skip processing if no IP and save location have been received
            if current_ip is None or current_save_location is None:
                # rospy.loginfo('No valid IP or save location received yet. Waiting...')
                rospy.sleep(1)  # Sleep for a bit
                continue

            rospy.loginfo('[communication] Waiting for a connection from ip %s' % current_ip)
            connection, client_address = sock.accept()
            try:
                rospy.loginfo('[communication] Connection from %s', client_address)
		
		if client_address[0].strip() == current_ip.strip():
                    # Send "ready to receive" message to the Raspberry Pi
                    ready_msg = "ready to receive"
                    connection.sendall(ready_msg.encode('utf-8'))
                    rospy.loginfo('[communication] Sent "ready to receive" message to %s', client_address)
		
		    received_data = False
                    with open(current_save_location, 'wb') as f:
                        while not rospy.is_shutdown():
			    ready = select.select([connection], [], [], 4) # 4 second timeout
			    if ready[0]:
                                data = connection.recv(1024)
                                if data:
                                    rospy.loginfo('[communication] Received chunk of size: %d' % len(data))
                                    f.write(data)
                                else:
                                    rospy.loginfo('[communication] Data collection complete')
				    received_data = True
                                    break
			    else:
				rospy.loginfo('[communication] Timeout reached. No data received in 4 seconds. Retrying...')
                                break

		    if received_data:
                        # Send acknowledgment message to the Raspberry Pi
                        ack_msg = "ack"
                        connection.sendall(ack_msg.encode('utf-8'))
                        rospy.loginfo('[communication] Sent "ack" message to %s', client_address)
                        
                        # Publish 'done' message once communication is complete
                        done_pub.publish(String("done"))
                        rospy.loginfo('[communication] Published "done" message')
                        current_ip = None
                        current_save_location = None
                    else:
                        rospy.loginfo('[communication] Data reception was incomplete, retrying...')
			continue

                else:
                    rospy.logwarn('[communication] Unexpected IP address: %s. Expected %s. Closing connection.', client_address[0], current_ip)
                    disconnect_msg = "go to sleep"
                    connection.sendall(disconnect_msg.encode('utf-8'))
	    finally:
                connection.close()
		rospy.loginfo("[communication] Connection closed")
    except Exception as e:
        rospy.logerr('[communication] Error: %s' % e)
    finally:
	if sock:
            sock.close()
	    rospy.loginfo("[communication] Socket closed")
	else:
	    rospy.logerr("[communication] Socket closed before finishing data collection.")

def callback(data):
    """
    Callback function for receiving IP and save location data.
    Parses the data and updates the global variables.
    """
    global current_ip, current_save_location
    try:
        ip, location = data.data.split(',')
	current_ip = str(ip).strip()
        current_save_location = str(location).strip()
	
        rospy.loginfo('[communication] Received IP: %s and save location: %s', current_ip, current_save_location)
    except Exception as e:
        rospy.logerr('[communication] Failed to process received message: %s', e)


if __name__ == '__main__':
    try:
        rospy.init_node('communication', anonymous=True)	
	signal.signal(signal.SIGINT, signal_handler)
	done_pub = rospy.Publisher("connection_done", String, queue_size=10)
	rospy.Subscriber("ip_save_location", String, callback)
        ensure_socket_closed(DRONE_IP, DRONE_PORT)

        # Start server as a thread
	server_thread = threading.Thread(target=start_server, args=(DRONE_IP, DRONE_PORT))
	server_thread.daemon = True
	server_thread.start()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass


