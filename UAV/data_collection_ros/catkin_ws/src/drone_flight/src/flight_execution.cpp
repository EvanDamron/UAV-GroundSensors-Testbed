/*
 * @file flight_execution.cpp
 *
 * Author: Evan Damron
 * This node sends all necessary mavlink messages to the drone. Once the drone is in offboard mode, it takes off. 
 * Then it listens for points from waypoint_manager.cpp. It traverses these points, then returns home and lands.
 * This version takes points as GPS coordinates.
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

#include <array>
#include <limits>

#define RATE 20  // loop rate hz

mavros_msgs::State current_state;
geographic_msgs::GeoPoseStamped position_home;
geographic_msgs::GeoPoseStamped current_target;
std::array<float, 3> next_point;
std::array<float, 3> last_point = {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()};
bool received_first_point = false;
bool received_second_point = false;
bool arrived_at_home = false;


/**
 * Converts latitude and longitude to a GeoPoseStamped message.
 * @param latlon Array containing latitude, longitude, and altitude.
 * @return A GeoPoseStamped message with the given latitude, longitude, and altitude.
 */
geographic_msgs::GeoPoseStamped latLonToPositionTarget(const std::array<float, 3>& latlon){
    
    geographic_msgs::GeoPoseStamped target;
    target.pose.position.latitude = latlon[0];
    target.pose.position.longitude = latlon[1];
    target.pose.position.altitude = latlon[2];

    return target;
}


/**
 * Callback for state updates from MAVROS.
 * @param msg Pointer to the received State message.
 */
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}


/**
 * Callback for receiving the next waypoint.
 * @param msg Pointer to the received Float32MultiArray message containing the next waypoint.
 */
void next_point_cb(const boost::shared_ptr<const std_msgs::Float32MultiArray>& msg) {
    if (msg->data.size() == 3) {
        std::array<float, 3> latlon = {msg->data[0], msg->data[1], msg->data[2]};
	//handle the takeoff position differently than the other waypoints
        if (!received_first_point){
	    received_first_point = true;
	    position_home = latLonToPositionTarget(latlon);
	    ROS_INFO("[flight execution] Received first point: lat = %f, lon = %f, height (MSL): %f", latlon[0], latlon[1], latlon[2]);
	}
	else {
	    received_second_point = true;
            next_point = latlon;
            ROS_INFO("[flight execution] Received next point: lat = %f, lon = %f, height (MSL): %f", next_point[0], next_point[1], next_point[2]);
        }
    }
    else {
        ROS_WARN("[flight execution] Received array does not contain exactly three elements.");
    }
}


/**
 * Callback for receiving the message that the drone has arrived back at the landing position.
 * @param msg Pointer to the received String message.
 */
void arrived_home_cb(const std_msgs::String::ConstPtr& msg){
    if (msg->data == "arrived")
    {
        arrived_at_home = true;
        ROS_INFO("[flight execution] Received arrived at home message");
    }
    else {
        ROS_INFO("[flight execution] Received unexpected message");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flight_execution");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Publisher target_global_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10);
    ros::Subscriber next_point_sub = nh.subscribe<std_msgs::Float32MultiArray>("next_point", 10, next_point_cb);
    ros::Subscriber arrived_home_sub = nh.subscribe<std_msgs::String>("arrived_home", 10, arrived_home_cb);

    ros::Rate rate(RATE);

    //Wait until the first point (where to takeoff to) has been received
    while (ros::ok() && !received_first_point) {
        ros::spinOnce();
        rate.sleep();
    }

    // Wait for the drone to connect to thee flight control unit (FCU)
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
	target_global_pub.publish(position_home);
        ROS_INFO_THROTTLE(1, "[flight execution] Connecting to FCU...");
    }
    ROS_INFO("[flight execution] Connected to FCU.");

    // Wait for the drone to enter offboard mode and arm
    ROS_INFO("[flight execution] waiting for offboard mode");
    while (ros::ok()) {
        target_global_pub.publish(position_home);
        ros::spinOnce();
        rate.sleep();
        if (current_state.mode == "OFFBOARD" && !current_state.armed) {
            ROS_INFO("[flight execution] Offboard mode detected.");
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("[flight execution] Vehicle armed");
            } else {
                ROS_ERROR("[flight execution] Failed to arm vehicle.");
                continue;
            }
            break;
        }
    }

    // Takeoff and hold the position for a bit
    int i = RATE * 6;
    ROS_INFO("[flight execution] taking off");
    while (ros::ok() && i > 0) {
        i--;
        target_global_pub.publish(position_home);
	ROS_INFO_THROTTLE(2.0, "[flight execution] Publishing global position target latitude : %.5f, longitude:%.5f, height: %.5f", position_home.pose.position.latitude, position_home.pose.position.longitude, position_home.pose.position.altitude);
        ros::spinOnce();
        rate.sleep();
    }
    
    // Publish setpoints for the received waypoints until 0,0 is received
    float landing_altitude;
    ROS_INFO("[flight execution] Traversing Waypoints");
    while (ros::ok()) {
        if (!received_second_point) {
            ROS_INFO("[flight execution] Haven't received a waypoint.");
            target_global_pub.publish(position_home);
            ros::spinOnce();
            rate.sleep();
            continue;
        }
        if (next_point[0] == 0 && next_point[1] == 0) {
            ROS_INFO("[flight execution] Received {0,0}. Time to stop traversing waypoints.");
            landing_altitude = next_point[2];
	    break;
        }

        if (next_point != last_point) {
            current_target = latLonToPositionTarget(next_point);
            last_point = next_point;
        }
	ROS_INFO_THROTTLE(2.0, "[flight execution] Publishing global position target latitude : %.5f, longitude:%.5f, height: %.5f", current_target.pose.position.latitude, current_target.pose.position.longitude, current_target.pose.position.altitude);
        target_global_pub.publish(current_target);
        ros::spinOnce();
        rate.sleep();
    }


    // Publish setpoints for landing position until the drone arrives there
    ROS_INFO("[flight execution] traveling back to landing position");
    while (ros::ok() && !arrived_at_home) {
        target_global_pub.publish(position_home);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = position_home.pose.position.latitude;
    land_cmd.request.longitude = position_home.pose.position.longitude;
    land_cmd.request.altitude = position_home.pose.position.altitude;

    ROS_INFO("[flight execution] Trying to land");
    while (!(land_client.call(land_cmd) && land_cmd.response.success)) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("[flight execution] Attempting to land again");
    }
    ROS_INFO("[flight execution] Landing command successful.");

    return 0;
}
