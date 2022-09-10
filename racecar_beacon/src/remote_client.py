#!/usr/bin/env python

import socket
from struct import *

HOST = '127.0.0.1'  # ros_monitor adress
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65432

# Call possible : RPOS, OBSF et RBID

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as rc: # AF_INET = IPv4, SOCK_STREAM = TCP
    rc.connect((HOST, PORT))

    print ("Enter the code you want to send :") # RPOS, OBSF or RBID
    msg = input()   

    rc.send(msg.encode("ASCII"))

    data = rc.recv(1024)    # Unpack msg
    
    print(data)
    rc.close()
    
# He is the one that is closing the connection, not ros_monitor.py