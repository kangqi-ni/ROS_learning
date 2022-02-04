This repo contains instructions to generate 3 turtles and control one of them to perform circular motion

The following commands are used in the terminal:
roscore

rosrun turtlesim turtlesim_node 

rosservice call /spawn "x: 3.0
y: 3.0
theta: 0.0
name: 'turtle2'" 

rosservice call /spawn "x: 7.0
y: 7.0
theta: 0.0
name: 'turtle3'" 

rqt_graph

rostopic pub -r 10 /turtle3/cmd_vel geometry_msgs/Twist "linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.8" 

rosrun rqt_plot rqt_plot
