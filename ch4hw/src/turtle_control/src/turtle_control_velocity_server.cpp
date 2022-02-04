#include "ros/init.h"
#include "ros/ros.h"
#include "turtle_control/ControlSrv.h"
#include "geometry_msgs/Twist.h"
#include <set>
#include <vector>
#include <map>

bool ControlCallback(turtle_control::ControlSrv::Request &req, 
                    turtle_control::ControlSrv::Response &res) {

    ros::NodeHandle n1;
    ros::Publisher pub = n1.advertise<geometry_msgs::Twist>("/"+req.name + "/cmd_vel", 10);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist msg;

    // Start circular motion
    if (req.start){

        msg.linear.x = req.linear_x;
        msg.angular.z = req.angular_z;
    }
    // Stop circular motion
    else {
        msg.linear.x = 0;
        msg.angular.z = 0;
    }

    while (ros::ok()) {
        ROS_INFO("vel_publish: linear_x=%f angular_z=%f", msg.linear.x, msg.angular.z);
        pub.publish(msg);
        loop_rate.sleep();
        ros::spinOnce();
    }

    // Feedback info
    res.success = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_server");

    ros::NodeHandle n;

    ros::ServiceServer person_service = n.advertiseService("/control_motion", ControlCallback);

    ROS_INFO("Ready to control motion.");
    ros::spin();

    return 0;
}