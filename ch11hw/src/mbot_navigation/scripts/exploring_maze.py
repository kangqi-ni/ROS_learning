#!/usr/bin/env python 
# -*- coding: utf-8 -*-

import rospy  
import actionlib  
import random
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from std_msgs.msg import Int8

STATUS_EXPLORING    = 0
STATUS_CLOSE_TARGET = 1
STATUS_GO_HOME      = 2

class ExploringMaze():
    def __init__(self):  
        rospy.init_node('exploring_maze', anonymous=True)  
        rospy.on_shutdown(self.shutdown)  

        # 在每个目标位置暂停的时间 (单位：s)
        self.rest_time = rospy.get_param("~rest_time", 0.5)  

        # 发布控制机器人的消息  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5) 
 
        # 创建一个Subscriber，订阅/exploring_cmd
        rospy.Subscriber("/exploring_cmd", Int8, self.cmdCallback)

        # 订阅move_base服务器的消息  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  

        rospy.loginfo("Waiting for move_base action server...")  
        
        # 60s等待时间限制  
        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server")  
 
        # 保存运行时间的变量   
        start_time = rospy.Time.now()  
        running_time = 0  

        rospy.loginfo("Starting exploring maze")  
        
        # 初始位置
        start_location = Pose(Point(0, 0, 0), Quaternion(0.000, 0.000, 0.709016873598, 0.705191515089))  
 
        # 命令初值
        self.exploring_cmd = 0
       
        # 开始主循环，随机导航  
        while not rospy.is_shutdown():
            # construct a goal
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = start_location
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()

            if self.exploring_cmd is STATUS_EXPLORING:
                # randomly generate a goal point
                self.goal.target_pose.pose.position.x = random.randint(0, 8)
                self.goal.target_pose.pose.position.y = random.randint(0, 9)
            elif self.exploring_cmd is STATUS_CLOSE_TARGET:
                # do not generate goal points
                rospy.sleep(0.1)
                continue
            elif self.exploring_cmd is STATUS_GO_HOME:
                # set goal point as the origin
                self.goal.target_pose.pose.position.x = 0
                self.goal.target_pose.pose.position.y = 0

            rospy.loginfo("going to: " + str(self.goal.target_pose.pose))
                
            # send the goal and nagivate
            self.move_base.send_goal(self.goal)
            # set time limit to 5 min
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal point!")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                else:
                    rospy.loginfo("Goal failed!")
            # convert running time to min
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60
            rospy.loginfo("Current time: " + str(trunc(running_time, 1)) + " min")

            if self.exploring_cmd is STATUS_GO_HOME:
                break
            else:
                rospy.sleep(self.rest_time)

        self.shutdown()

    def cmdCallback(self, msg):
        rospy.loginfo("Receive exploring cmd : %d", msg.data)
        self.exploring_cmd = msg.data

        if self.exploring_cmd is STATUS_CLOSE_TARGET:
            rospy.loginfo("Stopping the robot...")  
            self.move_base.cancel_goal() 

    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)  

def trunc(f, n):  
    slen = len('%.*f' % (n, f))  
    return float(str(f)[:slen])  

if __name__ == '__main__':  
    try:  
        ExploringMaze()  
        rospy.spin()  

    except rospy.ROSInterruptException:  
        rospy.loginfo("Exploring maze finished.")
