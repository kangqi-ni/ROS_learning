#include "ros/ros.h"
#include "turtlesim/Pose.h"

void PositionInfoCallback(const turtlesim::Pose::ConstPtr& msg) {
    // Print position information
    ROS_INFO("Turtle position info: x:%f y:%f", 
			 msg->x, msg->y);
}

int main(int argc, char **argv)
{
    // Inotialize ROS node "position_suscriber"
    ros::init(argc, argv, "position_suscriber"); 

    // Create node handler
    ros::NodeHandle n; 

    // Initialize subscriber that publishes to topic "/turtle1/pose" message type is turtlesim::Pose
    ros::Subscriber position_subscriber = n.subscribe<turtlesim::Pose>("/turtle1/pose", 10, PositionInfoCallback);

    ros::spin();

    return 0;
}