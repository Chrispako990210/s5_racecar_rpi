#!/usr/bin/env python

import socket
from struct import unpack

HOST = '127.0.0.1'  # ros_monitor adress
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65432

# Call possible : RPOS, OBSF et RBID

format_dict = {
    "RPOS": ">fffxxxx",
    "OBSF": ">Ixxxxxxxxxxxx",
    "RBID": ">Ixxxxxxxxxxxx"
}
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as rc: # AF_INET = IPv4, SOCK_STREAM = TCP
    rc.connect((HOST, PORT))
    try:
        while True:
            msg = input(">")
            rc.send(msg.encode("ASCII")) 
            data = rc.recv(16)
            format = format_dict.get(msg, '>16s')
            if format:
                print(unpack(format, data))
            if not data:
                break
    except KeyboardInterrupt:
        rc.close()
    
# He is the one that is closing the connection, not ros_monitor.py