/**
 * @file manager.cpp
 *
 * Author: Evan Damron
 * This node stores the location of the waypoints as float[x,y] coordinates, which are published to
 * flight_execution.cpp one at a time. The height (z) is specified in flight_execution.cpp. 
 * It also stores the ip address and save location associated to each sensor, which are published as a 
 * string to communication.py one at a time.  
 * 
 * 'manager' sends the first point immediately, and whenever the drone is within COMM_RADIUS
 * of the point, it sends the ip address and save location. Then the communication node handles the
 * data transfer, sending a notification when it has finished. Once all points and ip_save_location's, 
 * have been sent, manager sends [0,0] and the mission is complete.
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/HomePosition.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <cmath>
#include <vector>
#include <string>
#include <GeographicLib/Geoid.hpp>

#define COMM_RADIUS 3.0
#define DRONE_HEIGHT 5

struct SensorInfo {
    float latitude;
    float longitude;
    std::string ip;
    std::string save_location;
};

std::vector<SensorInfo> sensors;

/**
 * Reads the sensor configuration from a file.
 * @param file_path The path to the sensor configuration file.
 * @return A vector of SensorInfo structs containing the sensor data.
 */
std::vector<SensorInfo> readSensorConfig(const std::string& file_path) {
    std::vector<SensorInfo> temp_sensors;
    std::ifstream file(file_path);

    if (!file.is_open()) {
        std::cerr << "Failed to open config file: " << file_path << std::endl;
        return temp_sensors;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        SensorInfo sensor;
        char comma;

        ss >> sensor.latitude >> comma
           >> sensor.longitude >> comma
           >> std::ws;
        std::getline(ss, sensor.ip, ',');
        std::getline(ss, sensor.save_location, ',');
	ROS_INFO("Adding Sensor %.6f, %.6f, %s, %s", sensor.latitude, sensor.longitude, sensor.ip.c_str(), sensor.save_location.c_str());
        temp_sensors.push_back(sensor);
    }
    file.close();
    return temp_sensors;
}

size_t sensors_visited = 0;
bool collecting = false;   // True when currently collecting data from a sensor
bool home_position_set = false; // True when the home position of the drone has been received from mavros/home_position/home
bool arrived = false; // True when the drone has arrived at the landing position after completing the mission

ros::Publisher next_point_pub;
ros::Publisher ip_save_location_pub;
ros::Publisher arrived_home_pub;

float flying_altitude = 0.0;
float landing_altitude;

/**
 * Publishes the next waypoint or signals the completion of the mission if all sensors are visited.
 */
void check_and_publish_next_point() {
    if (sensors_visited >= sensors.size()) {
	ROS_INFO("[manager] All sensors have been visited. Returning to home");
        std_msgs::Float32MultiArray msg;
        msg.data = {0, 0, landing_altitude};
        next_point_pub.publish(msg);
        return;
    }
    
    ROS_INFO("[manager] Publishing next waypoint (%zu) - lat: %.7f, lon: %.7f, alt (MSL): %.7f", sensors_visited, sensors[sensors_visited].latitude, sensors[sensors_visited].longitude, flying_altitude);
    std_msgs::Float32MultiArray msg;
    msg.data = {sensors[sensors_visited].latitude, sensors[sensors_visited].longitude, flying_altitude};
    next_point_pub.publish(msg);
}


/**
 * Converts ellipsoidal height to mean sea level (MSL) height. mavros/home_position/home and mavros/global_position/global 
 * provide ellipsoidal height, but setpoint_position/global expects MSL, making this conversion necessary.
 * @param lat Latitude of the point.
 * @param lon Longitude of the point.
 * @param ellipsoid_alt Ellipsoidal altitude of the point.
 * @return Height above mean sea level.
 */
float ellipsoidToMSL(float lat, float lon, float ellipsoid_alt) {
    static GeographicLib::Geoid geoid("egm96-5");
    double geoid_height = geoid(lat, lon);
    return ellipsoid_alt - geoid_height;
}


/**
 * Callback for the home position topic. Uses this altitude to determine the flying and landing altitude of the drone.
 * @param msg Pointer to the received HomePosition message.
 */
void home_position_cb(const mavros_msgs::HomePosition::ConstPtr& msg) {
    if (!home_position_set){
        home_position_set = true;
        float home_MSL_alt = ellipsoidToMSL(msg->geo.latitude, msg->geo.longitude, msg->geo.altitude);
        landing_altitude = home_MSL_alt;
        ROS_INFO("[manager] Set landing altitude to %.2f MSL", landing_altitude);
        flying_altitude = DRONE_HEIGHT + home_MSL_alt;
        ROS_INFO("[manager] Set flying altitude to %.2f MSL", flying_altitude);
    }
    ROS_INFO_ONCE("[manager] Home position set to: [Lat: %.7f, Lon: %.7f, Alt (ellipsoidal): %.2f]",
             msg->geo.latitude, msg->geo.longitude, msg->geo.altitude);
}


/**
 * Computes the distance between two GPS coordinates using the Haversine formula.
 * @param lat1 Latitude of the first point.
 * @param lon1 Longitude of the first point.
 * @param lat2 Latitude of the second point.
 * @param lon2 Longitude of the second point.
 * @return Distance between the two points in meters.
 */
double haversine(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;

    lat1 = lat1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;

    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(lat1) * cos(lat2) *
               sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double R = 6371000; // Radius of Earth in meters
    return R * c;
}

/**
 * Callback for the current position topic.
 * @param msg Pointer to the received NavSatFix message.
 */
void current_position_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    // If all sensors are visited or currently collecting data, no need to calculate distance
    if (arrived || sensors_visited < 1 || collecting) return;

    // Extract the current latitude, longitude from the PoseStamped message
    double current_latitude = msg->latitude;
    double current_longitude = msg->longitude;

    // Retrieve the current target point
    SensorInfo& target_sensor = (sensors_visited < sensors.size()) ? sensors[sensors_visited] : sensors[0];

    // Compute the distance using the Haversine formula
    double distance = haversine(current_latitude, current_longitude, target_sensor.latitude, target_sensor.longitude);

    ROS_INFO_THROTTLE(3.0, "[manager] The drone is at Latitude: %.7f, Longitude: %.7f, Height (ellipsoidal): %.5f" , current_latitude, current_longitude, msg->altitude);
    ROS_INFO_THROTTLE(3.0, "[manager] The drone is %.5f meters from the target sensor", distance);

    // Check if within communication radius
    if (distance <= COMM_RADIUS) {
        if (sensors_visited >= sensors.size()){
            ROS_INFO("[manager] The drone has arrived at the landing position. Sending message to flight executioner.");
            arrived = true;
            std_msgs::String msg;
            msg.data = "arrived";
            arrived_home_pub.publish(msg);
        }
	ROS_INFO("[manager] The drone is within the communication radius of the sensor. Forwarding the IP and save location to 'communication'");
        collecting = true;
	// Publish port and save location
	std_msgs::String ip_save_location_msg;
	ip_save_location_msg.data = target_sensor.ip + "," + target_sensor.save_location;
	ip_save_location_pub.publish(ip_save_location_msg);

    }
}

/**
 * Callback for the connection done topic.
 * @param msg Pointer to the received String message.
 */
void connection_done_cb(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("[manager] Received 'connection done' topic");
    sensors_visited++;
    collecting = false;
    check_and_publish_next_point();
}


/**
 * Callback for the battery state topic. Prints battery info every 5 seconds.
 * @param msg Pointer to the received BatteryState message.
 */
void battery_state_cb(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    ROS_INFO_THROTTLE(5, "Battery State: Voltage: %.2fV, Current: %.2fA, Percentage: %.2f%%",
                      msg->voltage,
                      msg->current,
                      msg->percentage * 100);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "manager");
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");

    std::string config_file;
    if (!nh_param.getParam("sensor_config_file", config_file)) {
        ROS_ERROR("Failed to get param 'sensor_config_file'");
        return -1;
    }

    ROS_INFO("Reading in sensor configuration file %s", config_file.c_str());
    sensors = readSensorConfig(config_file);

    ros::Subscriber home_position_sub       = nh.subscribe<mavros_msgs::HomePosition>
					    ("mavros/home_position/home", 10, home_position_cb);
    ros::Subscriber current_position_sub    = nh.subscribe<sensor_msgs::NavSatFix>
                                            ("mavros/global_position/global", 10, current_position_cb);
    ros::Subscriber connection_done_sub     = nh.subscribe<std_msgs::String>
                                            ("connection_done", 10, connection_done_cb);
    ros::Subscriber battery_state_sub       = nh.subscribe<sensor_msgs::BatteryState>
                                            ("mavros/battery", 10, battery_state_cb);
    next_point_pub                          = nh.advertise<std_msgs::Float32MultiArray>
                                            ("next_point", 10);
    ip_save_location_pub                    = nh.advertise<std_msgs::String>
                                            ("ip_save_location", 10);
    arrived_home_pub                        = nh.advertise<std_msgs::String>
                                            ("arrived_home", 10); 

    // Ensure home position is set before starting the mission
    ros::Rate rate(10); // 10 Hz
    while (ros::ok() && !home_position_set) {
        ROS_INFO_THROTTLE(10, "[manager] Waiting for home position to be set...");
        ros::spinOnce();
        rate.sleep();
    }

    ros::Duration(1.0).sleep();

    // Immediately publish the first point
    check_and_publish_next_point();
    sensors_visited++;
    
    ros::Duration(1.0).sleep();
    check_and_publish_next_point();

    ros::spin();
    return 0;
}
