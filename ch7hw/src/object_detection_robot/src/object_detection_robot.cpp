#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "object_detection_robot/Pose.h"
 
int flag = 0;
long int width, height, x;

class DetectionToTwistConverter{
public:
    DetectionToTwistConverter(){
        pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        sub = n.subscribe("Pose", 1, &DetectionToTwistConverter::callback, this);
    }
 
    void callback(const object_detection_robot::Pose::ConstPtr& pose){
        // Store information for comparison
        if(flag == 0){
            width = pose->size_x;
            height = pose->size_y;
            x = pose->pos_x;
            flag = 1;
        }
        // Compare information
        else{
            // Cup becomes larger
            if((pose->size_x > width+20) && (pose->size_y > height+20)){
                ROS_INFO("Move forward");
                width = pose->size_x;
                height = pose->size_y;
                x = pose->pos_x;
                geometry_msgs::Twist msg;
                msg.linear.x = 0.2;
                msg.angular.z = 0;
                pub.publish(msg);
            }
            // Cup becomes smalles
            else if((pose->size_x < width-20) && (pose->size_y < height-20)){
                ROS_INFO("Move back");
                width = pose->size_x;
                height = pose->size_y;
                x = pose->pos_x;
                geometry_msgs::Twist msg;
                msg.linear.x = -0.2;
                msg.angular.z = 0;
                pub.publish(msg);
            }
            // Cup is to the right
            else if((pose->pos_x + pose->size_x/2) < (x + width/2 - 30)){
                ROS_INFO("Turn right");
                width = pose->size_x;
                height = pose->size_y;
                x = pose->pos_x;
                geometry_msgs::Twist msg;
                msg.linear.x = 0;
                msg.angular.z = -0.2;
                pub.publish(msg);
            }
            // Cup is to the left
            else if((pose->pos_x + pose->size_x/2) > (x + width/2 + 30)){
                ROS_INFO("Turn left");
                width = pose->size_x;
                height = pose->size_y;
                x = pose->pos_x;
                geometry_msgs::Twist msg;
                msg.linear.x = 0;
                msg.angular.z = 0.2;
                pub.publish(msg);
            }
        }
    }
 
private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
};
 
int main(int argc, char** argv){
    ros::init(argc, argv, "object_detection_robot");
    DetectionToTwistConverter SAPObject;
    ros::spin();
 
    return 0;
}