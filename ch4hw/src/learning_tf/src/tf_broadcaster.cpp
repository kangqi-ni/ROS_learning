#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_broadcaster");

    ros::NodeHandle node;

    tf::TransformBroadcaster broadcaster;

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        // Broadcast transformation from base_link to base_laser
        tf::Transform transform (tf::Quaternion(0,0,0,1), tf::Vector3(0.1,0.0,0.2));
        // base_link -> root, base_link -> leaf
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "base_laser"));
        loop_rate.sleep();
    }

    return 0;
}