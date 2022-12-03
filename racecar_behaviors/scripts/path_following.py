#!/usr/bin/env python

import rospy
import actionlib
from typing import Deque
from collections import deque
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Empty

import dynamic_reconfigure.client

from goal import Goal

class PathFollowing:
    def __init__(self):
        self.i = 0
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()
        #self.goals_stack = deque()
        self.goals_stack: Deque[Goal] = deque()
        self.init_goals()  
        rospy.loginfo("init")

        self.max_speed = rospy.get_param('~max_speed', 1)
        self.max_steering = rospy.get_param('~max_steering', 0.37) # Peut etre a modifier
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        self.ballon_sub = rospy.Subscriber('ballon_pose', PoseStamped, self.ballon_pose_callback, queue_size=1)
        self.start_sub = rospy.Subscriber('/racecar/start', Empty, self.start_callback, queue_size=1)

    def init_goals(self):
        self.start_goal_pose = PoseStamped()
        self.start_goal_pose.pose.position.x = 13.5
        self.start_goal_pose.pose.position.y = 2.1
        self.start_goal_pose.pose.position.z = 0.0
        self.start_goal_pose.pose.orientation.w = 1.0
        self.start_goal_pose.pose.orientation.x = 0.0
        self.start_goal_pose.pose.orientation.y = 0.0
        self.start_goal_pose.pose.orientation.z = 0.0
        self.start_goal = Goal("start_goal", self.start_goal_pose, 2)

        self.end_goal_pose = PoseStamped()
        self.end_goal_pose.pose.position.x = 0.0
        self.end_goal_pose.pose.position.y = 0.0
        self.end_goal_pose.pose.position.z = 0.0
        self.end_goal_pose.pose.orientation.w = 0.0
        self.end_goal_pose.pose.orientation.x = 0.0
        self.end_goal_pose.pose.orientation.y = 0.0
        self.end_goal_pose.pose.orientation.z = 0.0
        self.end_goal = Goal("end_goal", self.end_goal_pose, 2)

        self.goals_stack.append(self.start_goal)

    def start_callback(self, msg):
    
        rospy.loginfo("avancer?")
        while self.goals_stack:
            rospy.loginfo("in while")
            
            #debug
            j = 0
            for i in self.goals_stack:
                rospy.loginfo("name : %s, position : %f", i.name, j)
                j += 1

            result = self.movebase_client(self.goals_stack[-1])
            if result:
                rospy.loginfo("in result")
                d = rospy.Duration(self.goals_stack[-1].wait_time, 0)
                rospy.sleep(d)
                #self.goals_stack[-1].atGoal = True
                #self.goals_stack.pop()

            
            if self.start_goal.atGoal and (self.end_goal not in self.goals_stack) and not self.end_goal.atGoal:
                self.goals_stack.append(self.end_goal)

    def ballon_pose_callback(self, pose: PoseStamped):
        self.i = self.i + 1
        goal = Goal(f'ballon{self.i}', pose, 5)
        self.goals_stack.pop()
        self.goals_stack.append(goal)
        rospy.loginfo("added ballon goal %s", goal.name)
        self.client.cancel_goal()

        #debug
        j = 0
        for i in self.goals_stack:
            rospy.loginfo("name : %s, position : %f", i.name, j)
            j += 1


    def movebase_client(self, target: Goal):
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.pose = target.pose.pose
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.header.frame_id = "racecar/map"
        rospy.loginfo("create new goal %s", target.name)

        # Sends the goal to the action server.
        rospy.loginfo("send_goal %s", target.name)
        self.client.send_goal(goal)
        # Waits for the server to finish performing the action.
        wait = self.client.wait_for_result()
        rospy.loginfo("got result %s", target.name)
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
        # Result of executing the action
            return self.client.get_result() 

    def scan_callback(self, msg):
        # Because the lidar is oriented backward on the racecar, 
        # if we want the middle value of the ranges to be forward:
        #l2 = len(msg.ranges)/2;
        #ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]
        pass
        # twist = Twist()
        # twist.linear.x = self.max_speed
        # twist.angular.z = 0
           
        # self.cmd_vel_pub.publish(twist)
        
    def odom_callback(self, msg):
        #rospy.loginfo("Current speed = %f m/s", msg.twist.twist.linear.x)
        pass

def main():
    rospy.init_node('path_following')
    pF = PathFollowing()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

