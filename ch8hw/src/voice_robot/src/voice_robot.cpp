#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

class VoiceToTwistConverter {
public:
    VoiceToTwistConverter() {
        pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
        sub = n.subscribe("/voiceWords", 1000, &VoiceToTwistConverter::callback, this);
    }

    void callback(const std_msgs::String::ConstPtr &msg){
        const std::string command = msg->data;

        // Move forward
        if (command.find("前") != std::string::npos) {
            geometry_msgs::Twist msg;
            msg.linear.x = 0.2;
            msg.angular.z = 0;
            pub.publish(msg);
        }
        // Move backward
        else if (command.find("后") != std::string::npos) {
            geometry_msgs::Twist msg;
            msg.linear.x = -0.2;
            msg.angular.z = 0;
            pub.publish(msg);
        }
        // Turn left
        else if (command.find("左") != std::string::npos) {
            geometry_msgs::Twist msg;
            msg.linear.x = 0;
            msg.angular.z = 0.1;
            pub.publish(msg);
        }
        // Turn right
        else if (command.find("右") != std::string::npos) {
            geometry_msgs::Twist msg;
            msg.linear.x = 0;
            msg.angular.z = -0.2;
            pub.publish(msg);
        }
        // Stop
        else if (command.find("停") != std::string::npos) {
            geometry_msgs::Twist msg;
            msg.linear.x = 0;
            msg.angular.z = 0;
            pub.publish(msg);
        }
        else {
            ROS_INFO("Command not recognized!");
        }
    }

private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_control");
    VoiceToTwistConverter converter;
    ros::spin();

    return 0;
}