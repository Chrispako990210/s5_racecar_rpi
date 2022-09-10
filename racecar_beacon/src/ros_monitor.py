#!/usr/bin/env python3

from ast import Break
import rospy
import socket
import threading

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class ROSMonitor:
    def __init__(self):
        # Add your subscriber here (odom? laserscan?):
        self.sub_odom = rospy.Subscriber("odometry/filtered", Odometry, self.Odom_CB)
        self.sub_laser = rospy.Subscriber("scan", LaserScan, self.Scan_CB)

        # Current robot state:
        self.id = 0xFFFF
        self.pos = (0,0,0)
        self.obstacle = False

        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", 65432)
        self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", 65431)
        self.HOST = rospy.get_param("HOST", '127.0.0.1')

        # Thread for RemoteRequest handling:
        self.rr_thread = threading.Thread(target=self.rr_loop)

        print("ROSMonitor started.")

    def rr_loop(self):
        # Init your socket here :
        self.rr_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.rr_socket.bind((self.HOST, self.remote_request_port))
        print("binded, start listening")
        self.rr_socket.listen(1)
        (conn, addr) = self.rr_socket.accept()
        print("connected, adress :", addr)
        try:
            while True:
                data = conn.receive(1024)
                print("data")
                if not data:
                    break
                
            conn.close()
        except KeyboardInterrupt:
            print("closing socket")
            conn.close()
            

    def Odom_CB(self, msg):
        pass

    def Scan_CB(self, msg):
        pass

if __name__=="__main__":
    rospy.init_node("ros_monitor")

    node = ROSMonitor()

    rospy.spin()


