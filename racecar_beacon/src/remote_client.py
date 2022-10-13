#!/usr/bin/env python

import socket
from struct import *

HOST = '127.0.0.1'  # ros_monitor adress
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65432

format_dict = {
    "RPOS": ">fff4x",
    "OBSF": ">I12x",
    "RBID": ">I12x"
}

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as rc: # AF_INET = IPv4, SOCK_STREAM = TCP
    rc.connect((HOST, PORT))

    try:
        while True: 
            print ("Enter the code you want to send :") 
            msg = input()  

            rc.send(msg.encode("ASCII"))

            data = rc.recv(16)   
            format = format_dict.get(msg, "ASCII") 

            if format == "ASCII":
                print(data.decode("ASCII"))
            else:
                print(unpack(format, data))

    except KeyboardInterrupt:
        rc.close()

# He is the one that is closing the connection, not ros_monitor.py