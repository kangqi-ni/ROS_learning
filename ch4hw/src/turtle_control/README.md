This package allows users to spawn turtles in turtlesim and control their speed of circular motion

Terminal Commands:
roscore

rosrun turtlesim turtlesim_node

rosrun turtle_control turtle_control_spawn_client [turtle_name]

rosrun turtle_control turtle_control_velocity_server

rosrun turtle_control turtle_control_velocity_client [turtle_name] [start] [linear_x] [angular_z]