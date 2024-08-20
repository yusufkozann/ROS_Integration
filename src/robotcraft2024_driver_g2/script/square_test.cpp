#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"

// Global variables for storing sensor data
sensor_msgs::Range front_sensor_msg;
sensor_msgs::Range right_sensor_msg;
sensor_msgs::Range left_sensor_msg;

// Callback function for processing front sensor data
void front_scan_callback(const sensor_msgs::Range::ConstPtr& front_sensor);

// Callback function for processing right sensor data
void right_scan_callback(const sensor_msgs::Range::ConstPtr& right_sensor);

// Callback function for processing left sensor data
void left_scan_callback(const sensor_msgs::Rangex::ConstPtr& left_sensor);

// Function to check for potential collisions based on sensor data
void collisionLineCheck();

int main(int argc, char** argv)
{   
    // Initialize the ROS node
    ros::init(argc, argv, "square_test");
    ros::NodeHandle nh;
    
    // Define the publisher for velocity commands
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define subscribers for the infrared sensors
    ros::Subscriber ir_front_sensor_sub = nh.subscribe("/ir_front_sensor", 10, front_scan_callback);
    ros::Subscriber ir_right_sensor_sub = nh.subscribe("/ir_right_sensor", 10, right_scan_callback);
    ros::Subscriber ir_left_sensor_sub = nh.subscribe("/ir_left_sensor", 10, left_scan_callback);

    // Initialize command messages
    geometry_msgs::Twist move_cmd;
    geometry_msgs::Twist stop_cmd;

    // Set initial stop command values
    stop_cmd.linear.x = 0.0;
    stop_cmd.linear.z = 0.0;

    // Parameters for movement and rotation
    double linear_speed = 0.05; // Forward speed in meters per second
    double angular_speed = M_PI / 4; // Angular speed in radians per second (45 degrees per second)
    float side_length = 0.5; // Side length of the square in meters
    double turn_angle = M_PI / 2; // Angle for turning (90 degrees)

    // Set loop rate for the control loop
    ros::Rate loop_rate(10);
    int state = 0; // Initial state
    ros::Time start_time = ros::Time::now(); // Start time for state transitions

    // Main control loop
    while(ros::ok())
    {
        // Check for potential collisions
        collisionLineCheck();

        switch(state)
        {
            case 0: // Move forward
                move_cmd.linear.x = linear_speed;
                move_cmd.angular.z = 0.0;

                // Transition to the next state after covering the side length
                if ((ros::Time::now() - start_time).toSec() >= side_length / linear_speed)
                {
                    state = 1;
                    start_time = ros::Time::now();
                }
                break;

            case 1: // Stop the robot
                move_cmd = stop_cmd;

                // Transition to the next state after a short delay
                if ((ros::Time::now() - start_time).toSec() >= 1.0)
                {
                    state = 2;
                    start_time = ros::Time::now();
                }
                break;

            case 2: // Turn 90 degrees
                move_cmd.linear.x = 0.0;
                move_cmd.angular.z = angular_speed;

                // Transition to the next state after completing the turn
                if ((ros::Time::now() - start_time).toSec() >= turn_angle / angular_speed) 
                {
                    state = 3;
                    start_time = ros::Time::now();
                }
                break;

            case 3: // Stop the robot
                move_cmd = stop_cmd;

                // Transition back to moving forward after a short delay
                if ((ros::Time::now() - start_time).toSec() >= 1.0) {
                    state = 0;
                    start_time = ros::Time::now();
                }
                break;
        }

        // Publish the current velocity command
        cmd_vel_pub.publish(move_cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

// Function to check for potential collisions based on sensor data
void collisionLineCheck() 
{
    // Check front sensor data
    if (front_sensor_msg.range < 150) {
        ROS_WARN("Collision risk! The robot is %.2f meters from an obstacle on the front side", front_sensor_msg.range/1000);
    }

    // Check right sensor data
    if (right_sensor_msg.range < 150) {
        ROS_WARN("Collision risk! The robot is %.2f meters from an obstacle on the right side", right_sensor_msg.range/1000);
    }

    // Check left sensor data
    if (left_sensor_msg.range < 150) {
        ROS_WARN("Collision risk! The robot is %.2f meters from an obstacle on the left side", left_sensor_msg.range/1000);
    }
}

// Callback function to update front sensor data
void front_scan_callback(const sensor_msgs::Range::ConstPtr& front_sensor) 
{
    front_sensor_msg = *front_sensor;
}

// Callback function to update right sensor data
void right_scan_callback(const sensor_msgs::Range::ConstPtr& right_sensor) 
{
    right_sensor_msg = *right_sensor;
}

// Callback function to update left sensor data
void left_scan_callback(const sensor_msgs::Rangex::ConstPtr& left_sensor) 
{
    left_sensor_msg = *left_sensor;
}
