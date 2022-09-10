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
        self.pb_thread = threading.Thread(target=self.pb_loop)
        
        print("ROSMonitor started.")
        self.rr_thread.start()
        #self.pb_thread.start()

    def pb_loop(self):
        print("PositionBroadcast started")
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        pass

    def rr_loop(self):
        print("RemoteRequest started")
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((self.HOST, self.remote_request_port))
        while True:
            s.listen(1)
            conn, addr = s.accept()
            print("connected,  adress: ", addr)
            try:
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break
                    print(data.decode("ASCII"))
                    #msg = input(">")
                    #conn.send(msg.encode("UTF-8"))
                    if data.decode("ASCII") == 'abc':
                        reply = '123'
                        conn.send(reply.encode("ASCII"))
                    else:
                        reply2 = "jsp"
                        conn.send(reply2.encode("ASCII"))
            except KeyboardInterrupt:
                print("closing remote request")
                conn.close()
                break
            conn.close()
        
            

    def Odom_CB(self, msg):
        pass

    def Scan_CB(self, msg):
        pass

    def shutdown(self):
        self.rr_thread.join()

if __name__=="__main__":
    rospy.init_node("ros_monitor")

    node = ROSMonitor()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()


