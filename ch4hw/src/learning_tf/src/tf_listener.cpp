#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

void TransformPoint(const tf::TransformListener &listener) {
    // Create a point in base_laser frame to transform to base_link frame
    geometry_msgs::PointStamped laser_point;
    laser_point.header.frame_id = "base_laser";

    // Use most recent transformation info available;
    laser_point.header.stamp = ros::Time();

    // Assign random coordinate values [0.0, 5.0) to a the point
    laser_point.point.x = float(rand()) / float(RAND_MAX) * 5.0f;
    laser_point.point.y = float(rand()) / float(RAND_MAX) * 5.0f;
    laser_point.point.z = float(rand()) / float(RAND_MAX) * 5.0f;

    try {
        geometry_msgs::PointStamped base_point;
        listener.transformPoint("base_link", laser_point, base_point);
        ROS_INFO("laser point: (%.2f, %.2f, %.2f) base point: (%.2f, %.2f, %.2f)",
            laser_point.point.x, laser_point.point.y, laser_point.point.z,
            base_point.point.x, base_point.point.y, base_point.point.z);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("Receive an error in transformation: %s", ex.what());
        return;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_listener");

    ros::NodeHandle node;

    tf::TransformListener listener;

    srand(time(NULL));

    ros::Timer timer = node.createTimer(ros::Duration(1.0), boost::bind(&TransformPoint, boost::ref(listener)));

    ros::spin();
}