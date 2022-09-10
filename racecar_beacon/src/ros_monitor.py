#!/usr/bin/env python

import rospy
import socket
import threading
import time
from struct import *

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from tf.transformations import euler_from_quaternion
class ROSMonitor:
    def __init__(self):
        # Subscribers
        self.sub_odom = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.sub_laser = rospy.Subscriber("/scan", LaserScan, self.scan_cb)

        # Current robot state:
        self.id = 0xFFFF    #UINT32
        self.pos = (0,0,0)
        self.obstacle = False

        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", 65432)
        self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", 65431)

        # Thread for RemoteRequest handling and PositionBroadcast handling:
        self.rr_thread = threading.Thread(target=self.rr_loop)
        self.pb_thread = threading.Thread(target=self.pb_loop)

        # Start the threads:
        self.rr_thread.start()
        self.pb_thread.start()      # See when and where to close the threads!

        print("ROSMonitor started!")

    def odom_cb(self, data: Odometry):
        self.pos = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
        #self.pos = (data.pose.pose.position.x, data.pose.pose.position.y, self.quaternion_to_yaw(data.pose.pose.orientation))

    def scan_cb(self, data: LaserScan):
        if (data.ranges[0] < 1):
            self.obstacle = True
        else:
            self.obstacle = False

    def rr_loop(self):
        # RemoteRequest thread (TCP)

        HOST = "x.x.x.x"    # remote_client adress

        RPOS_format = ">fffxxxx"                # > = big-endien, f = float32 (4 octets), x = padding (1 octets) 
        OBSF_RBID_format = ">Ixxxxxxxxxxxx"     # I = unint32, x = padding     

        # Init your socket here :
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as rr_socket:  # AF_INET = IPv4, SOCK_STREAM = TCP
            rr_socket.bind((HOST, self.remote_request_port))

            rr_socket.listen(1)
            (conn, addr) = rr_socket.accept()
        
            with conn:
                print("Connected by", addr)

                while True:
                    data = conn.recv(1024)
                    if not data:
                        break

                    decoded_cmd = data.decode("ASCII")
                    print("Commande received =", decoded_cmd)
                    
                    swithc = {
                        "RPOS": pack(RPOS_format, self.pos[0], self.pos[1], self.pos[2]),
                        "OBSF": pack(OBSF_RBID_format, self.obstacle, self.id)
                    }
                    conn.sendall(swithc.get(decoded_cmd, "Invalid command".encode("ASCII")))

                    if decoded_cmd == "RPOS":
                        # send position
                        msg2send = pack(RPOS_format, self.pos[0], self.pos[1], self.pos[2])
                    elif decoded_cmd == "OBSF":
                        # send obstacle
                        msg2send = pack(OBSF_format, self.obstacle)
                    elif decoded_cmd == "RBID":
                        # send id
                        msg2send = pack(RBID_format, self.id)
                    else:
                        print("Error : unknown command")
                        msg2send = pack(cmd_format, "ERRO")

                    # Send msg to client
                    conn.send(msg2send)

    def pb_loop(self):
        # PositionBroadcast thread (UDP)

        HOST = "x.x.x.x"    # Broadcast adress
        format = "fffI"    # f = float32, I = unint32
       

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as pb_socket:  # AF_INET = IPv4, SOCK_DGRAM = UDP
            pb_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # Enable broadcasting mode
            pb_socket.bind((HOST, self.pos_broadcast_port))

            pb_socket.listen(1)
            (conn, addr) = pb_socket.accept()
            with conn:
                print("Connected by", addr)
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break
                    msg2send = pack(format, self.pos[0], self.pos[1], self.pos[2], self.id)
                    conn.send(msg2send)
                    time.sleep(1)


    def quaternion_to_yaw(self, quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return yaw

if __name__=="__main__":
    rospy.init_node("ros_monitor")

    node = ROSMonitor()

    rospy.spin()