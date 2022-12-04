#!/usr/bin/env python

import rospy
import actionlib
from typing import Deque
from collections import deque
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Empty, String

import dynamic_reconfigure.client

from goal import Goal

class PathFollowing:
    def __init__(self):
        rospy.loginfo("init")
        self.i = 0
        self.goals_stack = []
        # self.report_list = []
        self.report_path = '/home/catkin_ws/src/s5_racecar_rpi/racecar_behaviors/report/Report.txt'

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()

        self.init_goals()  

        self.max_speed = rospy.get_param('~max_speed', 1)
        self.max_steering = rospy.get_param('~max_steering', 0.37) # Peut etre a modifier
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.img_saver_pub = rospy.Publisher('img_report', String, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        self.ballon_sub = rospy.Subscriber('ballon_pose', PoseStamped, self.ballon_pose_callback, queue_size=1)
        self.start_sub = rospy.Subscriber('/racecar/start', Empty, self.start_callback, queue_size=1)

    def init_goals(self):
        start_goal_pose = PoseStamped()
        start_goal_pose.pose.position.x = 13.5     # 1.4870813332307862
        start_goal_pose.pose.position.y = 2.1      # 0.19644110658552927
        start_goal_pose.pose.position.z = 0.0
        start_goal_pose.pose.orientation.w = 1.0
        start_goal_pose.pose.orientation.x = 0.0
        start_goal_pose.pose.orientation.y = 0.0
        start_goal_pose.pose.orientation.z = 0.0
        start_goal = Goal("start_goal", start_goal_pose, 0)

        end_goal_pose = PoseStamped()
        end_goal_pose.pose.position.x = 0.0
        end_goal_pose.pose.position.y = 0.0
        end_goal_pose.pose.position.z = 0.0
        end_goal_pose.pose.orientation.w = -1.0
        end_goal_pose.pose.orientation.x = 0.0
        end_goal_pose.pose.orientation.y = 0.0
        end_goal_pose.pose.orientation.z = 0.0
        end_goal = Goal("end_goal", end_goal_pose, 0)

        self.goals_stack.append(start_goal)
        self.goals_stack.append(end_goal)
        


    def start_callback(self, msg):
        self.movebase_client(self.goals_stack[0])
        

    def ballon_pose_callback(self, pose: PoseStamped):

        rospy.loginfo("ballon detecter")
        goal = Goal("ballon", pose, 0)
        self.goals_stack.append(goal)
        self.movebase_client(goal)
        

    def done_callback(self, status, result):

        rospy.loginfo("done_callback")
        if self.goals_stack[-1].name == "ballon":
            rospy.sleep( rospy.Duration(self.goals_stack[0].wait_time, 0)) 
            rospy.loginfo("sleep done")  
        self.goals_stack.pop()

        if self.goals_stack:
            self.movebase_client(self.goals_stack[-1])
            pass


    def movebase_client(self, target: Goal):
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.pose = target.pose.pose
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.header.frame_id = "racecar/map"
        rospy.loginfo("create new goal %s", target.name)

        # Sends the goal to the action server.
        self.client.send_goal(goal, self.done_callback)
        rospy.loginfo("new goal sended")



    def scan_callback(self, msg):
        # Because the lidar is oriented backward on the racecar, 
        # if we want the middle value of the ranges to be forward:
        #l2 = len(msg.ranges)/2;
        #ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]

        # twist = Twist()
        # twist.linear.x = self.max_speed
        # twist.angular.z = 0
           
        # self.cmd_vel_pub.publish(twist)

        pass
        
    def odom_callback(self, msg):
        #rospy.loginfo("Current speed = %f m/s", msg.twist.twist.linear.x)
        pass

    def add_to_record(self, target: Goal):
        self.img_saver_pub.publish(target.name)
        # TODO: add fonction a Tony qui get le path dans le graph et creer le fichier.
        with open(self.report_path, 'a') as f:
            f.write(target)

def main():
    rospy.init_node('path_following')
    pF = PathFollowing()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

