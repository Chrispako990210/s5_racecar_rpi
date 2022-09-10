#!/usr/bin/env python

from email.policy import default
from unittest import case
import rospy
import socket
import threading

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion





def quaternion_to_yaw(quat):
# Uses TF transforms to convert a quaternion to a rotation angle around Z.
# Usage with an Odometry message: 
#   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw
class ROSMonitor:
    def __init__(self):
        # Add your subscriber here (odom? laserscan?):
        # self.sub_odom = rospy.Subcriber(...)
        # self.sub_laser = rospy.Subscriber(...)

        self.sub_odom = rospy.Subscriber("Odometry/filtered", Odometry, self.odom_cb)
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        # Current robot state:
        self.id = 0xFFFF
        self.pos = (0,0,0)
        self.obstacle = False

        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", 65432)
        self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", 65431)

        # Thread for RemoteRequest handling:
        self.rr_thread = threading.Thread(target=self.rr_loop)

        print("ROSMonitor started.")

   

    def odom_cb(self, data: Odometry):
        print("Got msg from /odometry/filtered")
        print("Pose: x = {}, y = {}, yaw = {}".format(data.pose.pose.position.x, data.pose.pose.position.y, quaternion_to_yaw(data.pose.pose.orientation)))

    def scan_cb(self, data: LaserScan):
        print("Got msg from /scan")
        print("Ranges: {}".format(data.ranges))

    def rr_loop(self):
        HOST = '127.0.0.1'
        PORT = 65432

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((HOST, PORT))

        s.listen(1)
        (conn, addr) = s.accept() # returns new socket and addr. client
        while True: # forever
            data = conn.recv(1024) # receive data from client
            print(data)
            if not data: break # stop if client stopped
            conn.send(data) # return sent data plus an "*"
        conn.close() # close the connection

    def send_pos(self):
        # Send the current position to the remote request port.
        # Return a string with the current position.
        return "message sended"

    def send_obs(self):
        # Send the current obstacle state to the remote request port.
        # Return a string with the current obstacle state.
        return "message sended"
    
    def send_id(self): 
        # Send the current id to the remote request port.
        # Return a string with the current id.
        return "message sended"

    def read_cmd(self,data):
        # Read a command from the remote request port.
        # Return a string with the command.
        return data.decode('ascii')

    def associated_msg_cmd(self, msg):
        # Send a message to the remote request port.
        # Return a string with the message.
        
        switch={
            'RPOS': self.send_pos(),
            'OBSF': self.send_obs(),
            'RFBID': self.send_id(),
        }
        return switch.get(data_decode, "Invalid message")

        

if __name__=="__main__":
    rospy.init_node("ros_monitor")

    node = ROSMonitor()

    rospy.spin()


