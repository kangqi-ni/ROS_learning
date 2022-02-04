#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
    // Inotialize ROS node "velocity_publisher"
    ros::init(argc, argv, "velocity_publisher"); 

    // Create node handler
    ros::NodeHandle n; 

    // Initialize publisher that publishes to topic "/turtle1/cmd_vel" message type is geometry_msgs/Twist
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	
    // Set loop rate as 10 Hz (10 publication per second)
    ros::Rate loop_rate(10);

    while (ros::ok()){
        // Initialize message data 
        geometry_msgs::Twist msg;
        msg.linear.x = 2.0;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 1.8;

        // Publish message
        ROS_INFO("linear x: %lf; angular z: %lf", msg.linear.x, msg.angular.z);
        velocity_publisher.publish(msg);

        // Sleep according to loop rate
        loop_rate.sleep();
    }

    return 0;
}