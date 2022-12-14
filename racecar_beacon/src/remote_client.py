#!/usr/bin/env python3

import socket
from struct import unpack

HOST = '127.0.0.1'  # ros_monitor adress
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65432

format_dict = {
    "RPOS": ">fff4x",
    "OBSF": ">I12x",
    "RBID": ">I12x"
}

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
            format = format_dict.get(msg, 'utf8')
            if format == 'utf8':
                print(data.decode(format))
            else:
                print(unpack(format, data))
            if not data:
                break
    except KeyboardInterrupt:
        rc.close()
# He is the one that is closing the connection, not ros_monitor.py