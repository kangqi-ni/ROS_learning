#include "ros/ros.h"
#include "turtle_control/ControlSrv.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
    // Initialize ros node "control_client"
    ros::init(argc, argv, "control_client");

    // Construct node handle
    ros::NodeHandle n;

    // Construct a client to service "/control_motion", service type is turtle_control::ControlSrv
    ros::ServiceClient control_client = n.serviceClient<turtle_control::ControlSrv>("/control_motion");

    // Construct a service turtle_control::ControlSrv
    turtle_control::ControlSrv srv;
    srv.request.name = argv[1];
    srv.request.start = (strcmp(argv[2], "start") == 0);
    if (srv.request.start) {
        srv.request.linear_x = atof(argv[3]);
        srv.request.angular_z = atof(argv[4]);
    }

    // Wait until a service is called
    control_client.call(srv);
    
    // Print service content
    if (srv.request.start) {
        ROS_INFO("%s starts circular motion! linear_x = %f angular_z = %f", 
            srv.request.name.c_str(), srv.request.linear_x, srv.request.angular_z);
    }
    else {
        ROS_INFO("%s has been stopped", srv.request.name.c_str());
    }

    return 0;
}
