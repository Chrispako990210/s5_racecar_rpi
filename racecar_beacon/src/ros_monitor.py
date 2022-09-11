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
        self.HOST = rospy.get_param("HOST", "127.0.0.1")

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
        self.obstacle = min(data.ranges) <= 1.0

        # if (data.ranges >= 1):    # Check if that works
        #     self.obstacle = True
        # else:
        #     self.obstacle = False

    def RPOS_response(self):
        RPOS_format = ">fff4x"                # > = big-endien, f = float32 (4 octets), x = padding (1 octets) 
        msg2send = pack(RPOS_format, self.pos[0], self.pos[1], self.pos[2])
        return msg2send

    def OBSF_response(self):
        OBSF_format = ">I12x"     # I = unint32, x = padding
        msg2send = pack(OBSF_format, self.obstacle)
        return msg2send

    def RBID_response(self):
        RBID_format = ">I12x"     # I = unint32, x = padding
        msg2send = pack(RBID_format, self.id)
        return msg2send

    # def invalid_response(self):
    #     # invalid_format = ">1s15x"
    #     # msg2send = pack(invalid_format, "Invalid command")  # See if it works
    #     return "->Invalid command, try again".encode("utf8")

    def rr_loop(self):
        # RemoteRequest thread (TCP)
        dict_format = {
            "RPOS": self.RPOS_response,
            "OBSF": self.OBSF_response,
            "RBID": self.RBID_response
        }

        # Init your socket here :
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as rr_socket:  # AF_INET = IPv4, SOCK_STREAM = TCP
            rr_socket.bind((self.HOST, self.remote_request_port))
            while True:
                rr_socket.listen(1)
                (conn, addr) = rr_socket.accept()

                with conn:
                    print("Connected by : ", addr)
                    try:
                        while True:
                            data = conn.recv(16)
                            if not data:
                                break

                            decoded_cmd = data.decode("ASCII")
                            print("Commande received from client : ", decoded_cmd)

                            if decoded_cmd not in dict_format:
                                conn.send("ERROR : Invalid command, please try again".encode("ASCII"))
                            else:
                                conn.send(dict_format.get(decoded_cmd)())

                    except KeyboardInterrupt:
                        print("KeyboardInterrupt")
                        conn.close()
                        break
                    conn.close()


    def pb_loop(self):
        # PositionBroadcast thread (UDP)
        broadcast_adresss = "127.0.0.255"
        format = ">fffI"    # f = float32, I = unint32

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as pb_socket:  # AF_INET = IPv4, SOCK_DGRAM = UDP
            pb_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # Enable broadcasting mode
            # pb_socket.bind((broadcast_adresss, self.pos_broadcast_port))

            while True:
                # print("Broadcasting from ROSMonitor")
                msg2send = pack(format, self.pos[0], self.pos[1], self.pos[2], self.id)
                print(unpack(format, msg2send))
                # pb_socket.send(msg2send)
                pb_socket.sendto(msg2send, (broadcast_adresss, self.pos_broadcast_port))
                time.sleep(1)


    def quaternion_to_yaw(self, quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return yaw


    def shutdown(self):
        # Close the threads:
        self.rr_thread.join()
        self.pb_thread.join()


if __name__ == "__main__":
    rospy.init_node("ros_monitor")
    node = ROSMonitor()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()