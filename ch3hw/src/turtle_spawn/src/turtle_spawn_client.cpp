#include "ros/ros.h"
#include "turtlesim/Spawn.h"

int main(int argc, char **argv)
{
    // Initialize ros node "turtle_spawn"
    ros::init(argc, argv, "turtle_spawn");

    // Construct node handle
    ros::NodeHandle n;

    // Construct a client, service type is "turtlesim::Spawn"
    ros::service::waitForService("/spawn");
    ros::ServiceClient turtle_spawn_client = n.serviceClient<turtlesim::Spawn>("/spawn");

    // Construct a service turtlesim::Spawn
    turtlesim::Spawn srv;
	
    srand(time(NULL));
    srv.request.name = "turtle" + std::to_string(rand());

    // Generate random float between 0.0 and 10.0
    srv.request.x = float(rand()) / float(RAND_MAX) * 10.0f;
    srv.request.y = float(rand()) / float(RAND_MAX) * 10.0f;
    // Generate random float between 0.0 and 2*pi
    srv.request.theta = float(rand() % 360) * M_PI / 180.0f;

    // Wait until a service is called
    if (!turtle_spawn_client.call(srv)) {
        ROS_ERROR("Failed to call spawn service.");
    }
    
    ROS_INFO("%s is spawned at (%f, %f)", srv.response.name.c_str(), srv.request.x, srv.request.y);

    return 0;
}
