#!/usr/bin/env python3

from ast import Break
import rospy
import socket
import threading
import time
from struct import *

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

        # Command dictionnary for RemoteRequest
        self.dict_format = {
            "RPOS": self.RPOS_response,
            "OBSF": self.OBSF_response,
            "RBID": self.RBID_response
        }

        # Messages formats
        self.RPOS_format = ">fff4x"     # > = big-endien, f = float32 (4 octets), x = padding (1 octets) 
        self.OBSF_format = ">I12x"      # I = unint32, x = padding
        self.RBID_format = ">I12x"      # I = unint32, x = padding
        self.PB_format = ">fffI"        # f = float32, I = unint32

        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", 65432)
        self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", 65431)
        self.HOST = rospy.get_param("HOST", '127.0.0.1')

        # Thread for RemoteRequest handling:
        self.rr_thread = threading.Thread(target=self.rr_loop)
        self.pb_thread = threading.Thread(target=self.pb_loop)
        
        #Starting threads
        self.rr_thread.start()
        self.pb_thread.start()

        print("ROSMonitor started.")

    def pb_loop(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        print("PositionBroadcast started")
        while True:
            # Creating position message
            msg = pack(self.PB_format, self.pos[0], self.pos[1], self.pos[2], self.id)

            # Sending message
            s.sendto(msg, ('<broadcast>', self.pos_broadcast_port))

            # Waiting 1 sec for 1Hz broadcast frequency
            time.sleep(1)

    def rr_loop(self):     
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((self.HOST, self.remote_request_port))
        print("RemoteRequest started")
        s.listen(1)
        while True:
            conn, addr = s.accept()
            try:
                while True:
                    data = conn.recv(16)

                    #Break if connection is lost
                    if not data:
                        break

                    #decode command
                    decoded_cmd = data.decode("ASCII")
                    print("Received command : ", decoded_cmd)

                    #Reply if command is valid
                    if decoded_cmd not in self.dict_format:
                        conn.send("Error : Invalid command".encode("ASCII"))
                    else:
                        conn.send(self.dict_format.get(decoded_cmd)())

            except KeyboardInterrupt:
                conn.close()
                break
            conn.close()
        
    def Odom_CB(self, msg: Odometry):
        self.pos = (msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.orientation.z)
        
    def Scan_CB(self, msg: LaserScan):
        if msg.range_min <= 1:
            self.obstacle = True
        else:
            self.obstacle = False

    def RPOS_response(self): 
        msg2send = pack(self.RPOS_format, self.pos[0], self.pos[1], self.pos[2])
        return msg2send

    def OBSF_response(self):   
        msg2send = pack(self.OBSF_format, self.obstacle)
        return msg2send

    def RBID_response(self):     
        msg2send = pack(self.RBID_format, self.id)
        return msg2send

    def shutdown(self):
        self.rr_thread.join()
        self.pb_thread.join()

if __name__=="__main__":
    rospy.init_node("ros_monitor")
    try:
        node = ROSMonitor()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.on_shutdown(node.shutdown)

