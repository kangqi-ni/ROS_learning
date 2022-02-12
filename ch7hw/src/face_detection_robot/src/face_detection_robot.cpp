#include "ros/ros.h"
#include "face_detection_robot/Size.h"
#include "geometry_msgs/Twist.h"
 
int flag = 0;
int size_width, size_height, size_x;
 
class DetectionToTwistConverter{
public:
    DetectionToTwistConverter(){
        pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        sub = n.subscribe("Size", 1, &DetectionToTwistConverter::callback, this);
    }
 
    void callback(const face_detection_robot::Size::ConstPtr& size){
        // Store information for comparison
        if(flag == 0){
            size_width = size->width;
            size_height = size->height;
            size_x = size->x;
            flag = 1;
        }
        // Compare information
        else{
            // Face becomes larger
            if((size->width > size_width+50) && (size->height > size_height+50)){
                ROS_INFO("Move forward");
                //ROS_INFO("size->width:%d size_width:%d", size->width, size_width.data);
                size_width = size->width;
                size_height = size->height;
                size_x = size->x;
                geometry_msgs::Twist msg;
                msg.linear.x = 0.2;
                msg.angular.z = 0;
                pub.publish(msg);
            }
            // Face becomes smaller
            else if((size->width < size_width-50) && (size->height < size_height-50)){
                ROS_INFO("Move back");
                size_width = size->width;
                size_height = size->height;
                size_x = size->x;
                geometry_msgs::Twist msg;
                msg.linear.x = -0.2;
                msg.angular.z = 0;
                pub.publish(msg);
            }
            // Face is to the right
            else if((size->x + size->width) < (size_x + size_width - 50)){
                ROS_INFO("Turn right");
                size_width = size->width;
                size_height = size->height;
                size_x = size->x;
                geometry_msgs::Twist msg;
                msg.linear.x = 0;
                msg.angular.z = -0.2;
                pub.publish(msg);
            }
            // Face is to the left
            else if((size->x + size->width/2) > (size_x + size_width/2 + 50)){
                ROS_INFO("Turn left");
                size_width = size->width;
                size_height = size->height;
                size_x = size->x;
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
 
int main(int argc, char **argv){
  ros::init(argc, argv, "robot_control");
  DetectionToTwistConverter converter;
  ros::spin();
 
  return 0;
}
