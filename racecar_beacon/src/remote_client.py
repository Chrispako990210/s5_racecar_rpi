#!/usr/bin/env python

import socket

HOST = '10.0.1.21' 
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
try:
    while True:
        msg = input("Enter Code :")
        s.send(msg.encode("ASCII")) 
        data = s.recv(1024)
        if not data:
            break
        print(data.decode("ASCII"))
except KeyboardInterrupt:    
    s.close()
