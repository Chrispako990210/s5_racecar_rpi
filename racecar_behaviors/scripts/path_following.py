#!/usr/bin/env python

import rospkg
import rospy
import os
import actionlib
from geometry_msgs.msg import Twist, PoseStamped, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Empty, String
from blob_planner import *
from nav_msgs.srv import GetMap



# import dynamic_reconfigure.client

from goal import Goal

class PathFollowing:
    def __init__(self):
        rospy.loginfo("init")
        self.i = 0
        self.j = 0
        self.goals_stack = []
        rospack=rospkg.RosPack()

        rospack=rospkg.RosPack()
        pckg_path=rospack.get_path('racecar_behaviors')
        self.report_path = os.path.join(pckg_path, "report/Report.txt")

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()
        rospy.loginfo("map init begin")
        
        rospy.loginfo("map init success")
        self.max_speed = rospy.get_param('~max_speed', 1)
        self.start_goal_x  = rospy.get_param('~start_goal_x', 10.0)
        self.start_goal_y  = rospy.get_param('~start_goal_y', 10.0)
        self.start_goal_w  = rospy.get_param('~start_goal_w', 1.0)
        self.max_steering = rospy.get_param('~max_steering', 0.37) # Peut etre a modifier
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        self.ballon_sub = rospy.Subscriber('ballon_pose', PoseStamped, self.ballon_pose_callback, queue_size=1)

        self.debris_sub=rospy.Subscriber('position_debris', Pose,self.debris_pos_callback ,queue_size=1)

        self.start_sub = rospy.Subscriber('/racecar/start', Empty, self.start_callback, queue_size=1)
        self.img_saver_pub = rospy.Publisher('img_report', String, queue_size=1)
        self.init_goals()  
        self.map=init_map(0.6)
    #return array map_securitaire(brushfire), map_background (original), coord origine, resolution
    # def init_map(self,distSecuritaire):
    #     # rospy.init_node('brushfire')
    #     rospy.loginfo("init map")
    #     #get map
    #     prefix = "racecar"
    #     rospy.wait_for_service('get_map')
    #     try:
    #         get_map = rospy.ServiceProxy('get_map', GetMap)
    #         response = get_map()
    #     except (rospy.ServiceException) as e:
    #         print("Service call failed: %s"%e)
    #         return
        
    #     rospy.loginfo("Got map=%dx%d resolution=%f", response.map.info.height, response.map.info.width, response.map.info.resolution)    
    #     grid = np.reshape(response.map.data, [response.map.info.height, response.map.info.width])
    #     map_origin=(response.map.info.origin.position.x,response.map.info.origin.position.y)
        
    #     #copie de la carte originale
    #     background_map = grid.copy()
    #     maskFree=background_map==0
    #     background_map[maskFree] = 255 # free cells

    #     # distance securitaire en m
    #     map_resolution=response.map.info.resolution
    #     nbCouche=int(distSecuritaire/response.map.info.resolution)

    #     #augmentation de la grosseur des obstacle en appliquant brushfire
    #     map_brushfire = brushfire(grid)
    #     layerFire(map_brushfire,nbCouche)
    #     rospy.loginfo("map_brushfire=%dx%d resolution=%f", map_brushfire.shape[0], map_brushfire.shape[1])
    #     return (map_brushfire,background_map,map_origin,map_resolution)

    def init_goals(self):
        start_goal_pose = PoseStamped()
        start_goal_pose.header.frame_id = 'racecar/map'
        start_goal_pose.pose.position.x = self.start_goal_x     # 1.4870813332307862
        start_goal_pose.pose.position.y = self.start_goal_y    # 0.19644110658552927
        start_goal_pose.pose.position.z = 0.0
        start_goal_pose.pose.orientation.w = self.start_goal_w
        start_goal_pose.pose.orientation.x = 0.0
        start_goal_pose.pose.orientation.y = 0.0
        start_goal_pose.pose.orientation.z = 0.0
        start_goal = Goal("start_goal", start_goal_pose, 0)

        end_goal_pose = PoseStamped()
        end_goal_pose.header.frame_id = 'racecar/map'
        end_goal_pose.pose.position.x = 0.0
        end_goal_pose.pose.position.y = 0.0
        end_goal_pose.pose.position.z = 0.0
        end_goal_pose.pose.orientation.w = 0.0
        end_goal_pose.pose.orientation.x = 0.0
        end_goal_pose.pose.orientation.y = 0.0
        end_goal_pose.pose.orientation.z = 1.0
        end_goal = Goal("end_goal", end_goal_pose, 0)

        self.goals_stack.append(end_goal)
        self.goals_stack.append(start_goal)
        


    def start_callback(self, msg):
        
        rospy.loginfo("map receveived")
        self.movebase_client(self.goals_stack[-1])
        
    
    def debris_pos_callback(self, pose: Pose):
        name=f"ballon{self.j}.bmp"
        generate_path(pose,self.map,name)
        self.j+=1
        
    def ballon_pose_callback(self, pose: PoseStamped):

        rospy.loginfo("ballon detecter")
        goal = Goal(f"ballon{self.i}", pose, 5)
        self.goals_stack.append(goal)
        self.i = self.i + 1
        self.movebase_client(goal)
         

        

    def done_callback(self, status, result):

        rospy.loginfo("done_callback")
        if "ballon" in self.goals_stack[-1].name:
            rospy.sleep(5.0) 
            rospy.loginfo("Adding obstacle to record")
            self.add_to_record(self.goals_stack[-1])
        self.goals_stack.pop()

        if self.goals_stack:
            self.movebase_client(self.goals_stack[-1])
            pass

        


    def movebase_client(self, target: Goal):
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose = target.pose
        goal.target_pose.header.stamp = rospy.Time.now() 
        rospy.loginfo("create new goal %s", target.name)

        # Sends the goal to the action server.
        self.client.send_goal(goal, self.done_callback)
        
    def add_to_record(self, target: Goal):
        rospy.loginfo("start writting record")
        self.img_saver_pub.publish(target.picture_name)
        # TODO: add funct a Tony, prendre x, y du goal et le path name pour generer le .bmp
        answ = os.path.exists(self.report_path)
        with open(self.report_path, "a" if answ else "w") as f:
            f.write(str(target))
        rospy.loginfo("sucess writting record")

    def scan_callback(self, msg):
        # Because the lidar is oriented backward on the racecar, 
        # if we want the middle value of the ranges to be forward
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


def main():
    rospy.init_node('path_following')
    pF = PathFollowing()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

