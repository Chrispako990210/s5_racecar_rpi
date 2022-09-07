#!/usr/bin/env python

import rospy
import socket
import threading

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class ROSMonitor:
    def __init__(self):
        # Add your subscriber here (odom? laserscan?):
        self.sub_odom = rospy.Subcriber('odometry/filtered', Odometry, self.odom_cb)
        # self.sub_laser = rospy.Subscriber(...)

        # Current robot state:
        self.id = 0xFFFF
        self.pos = (0,0,0)
        self.obstacle = False

        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", 65432)
        self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", 65431)

        # Thread for RemoteRequest handling:
        self.rr_thread = threading.Thread(target=self.rr_loop)
        
        # Thread for PositionBroadcast handling:
        self.pb_thread = threading.Thread(target=self.pb_loop)

        print("ROSMonitor started.")

    def odom_cb(self, data: Odometry):
        pass

    def rr_loop(self):
        HOST = '10.0.1.4' # Valeur bidon, a linker plus tard
        # Init your socket here :
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as rr_socket: # Inits the socket with (Address family-> IPv4, Transport protocol-> TCP)
            rr_socket.bind((HOST, self.remote_request_port))
            rr_socket.listen()
            conn, addr = rr_socket.accept()
            with conn:
                # Define message format here ! See annexe guide etudiant
                print(f"Connected by {addr}")
                while True:
                    data = conn.recv(1024) # change reception size depending on format
                    if not data:
                        break
                    conn.sendall(data)

    def pb_loop(self):
        HOST = '10.0.1.4' # Valeur bidon, a linker plus tard
        # Init your socket here :
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as pb_socket: # Inits the socket with (Address family-> IPv4, Transport protocol-> UDP)
            pb_socket.bind((HOST, self.pos_broadcast_port)) # Might be better to change the pos_prodcast definition with vehicule tracker
            pb_socket.listen()
            conn, addr = pb_socket.accept()
            with conn:
                # Define message format here ! See annexe guide etudiant
                print(f"Connected by {addr}")
                while True:
                    data = conn.recv(1024) # change reception size depending on format
                    if not data:
                        break
                    conn.send(data)

if __name__=="__main__":
    rospy.init_node("ros_monitor")

    node = ROSMonitor()

    rospy.spin()


