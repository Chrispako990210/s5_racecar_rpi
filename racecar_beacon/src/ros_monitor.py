#!/usr/bin/env python3

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
        self.HOST = rospy.get_param("HOST", '127.0.0.1')
        # Thread for RemoteRequest handling and PositionBroadcast handling:
        self.rr_thread = threading.Thread(target=self.rr_loop)
        self.pb_thread = threading.Thread(target=self.pb_loop)
        # Start the threads:
        self.rr_thread.start()
        self.pb_thread.start()
        print("ROSMonitor started!")

    def odom_cb(self, data: Odometry):
        self.pos = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
        #self.pos = (data.pose.pose.position.x, data.pose.pose.position.y, self.quaternion_to_yaw(data.pose.pose.orientation))

    def scan_cb(self, data: LaserScan):
        if min(data.ranges) <= 1:
            self.obstacle = True
        else:
            self.obstacle = False

    def RPOS_response(self):
        RPOS_format = ">fffxxxx"                # > = big-endien, f = float32 (4 octets), x = padding (1 octets), ici 16 Bytes
        return pack(RPOS_format, self.pos[0], self.pos[1], self.pos[2])

    def OBSF_response(self):
        OBSF_format = ">Ixxxxxxxxxxxx"     # I = unint32, x = padding
        return pack(OBSF_format, self.obstacle)

    def RBID_response(self):
        RBID_format = ">Ixxxxxxxxxxxx"     # I = unint32, x = padding
        return pack(RBID_format, self.id)

    def rr_loop(self):
        # RemoteRequest thread (TCP)     
        msg2client = {
            "RPOS": self.RPOS_response,
            "OBSF": self.OBSF_response,
            "RBID": self.RBID_response
        }
        # Init your socket here :
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as rr_socket:  # AF_INET = IPv4, SOCK_STREAM = TCP
            rr_socket.bind((self.HOST, self.remote_request_port))
            while not rospy.is_shutdown():
                rr_socket.listen(1)
                (conn, addr) = rr_socket.accept()
                with conn:
                    print("Connected by", addr)
                    while True:
                        data = conn.recv(16)
                        if not data:
                            print("Disconnected by", addr)
                            conn.close()
                            break
                        decoded_cmd = data.decode("ASCII")
                        print("Commande received =", decoded_cmd)
                        # Send msg to client
                        msg = msg2client.get(decoded_cmd, None)
                        if msg is None:
                            conn.send("ERROR".encode("utf8"))
                        else:
                            conn.send(msg())

    def pb_loop(self):
        # PositionBroadcast thread (UDP)
        broadcast_addr = "127.0.0.255"
        format = ">fffI"    # f = float32, I = unint32
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as pb_socket:  # AF_INET = IPv4, SOCK_DGRAM = UDP
            pb_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # Enable broadcasting mode
            pb_socket.settimeout(0.2) # Empeche le blockage lors de reception
            print("brodcasting position...")
            while True:
                    msg2send = pack(format, self.pos[0], self.pos[1], self.pos[2], self.id)
                    # print("msg2send =", unpack(format, msg2send)) #for debugging 
                    pb_socket.sendto(msg2send, (broadcast_addr, self.pos_broadcast_port))
                    rospy.sleep(1)

    def quaternion_to_yaw(self, quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return yaw

    def shutdown(self):
        # Close the threads
        self.rr_thread.join()
        self.pb_thread.join()

if __name__=="__main__":
    try:
        rospy.init_node("ros_monitor")
        node = ROSMonitor()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROSMonitor")
        node.shutdown()