#!/usr/bin/env python

import socket

HOST = '10.0.1.21' 
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
try:
    while True: # forever
        msg = input("Enter code : ") # send data to client
        s.send(str(msg)) # return sent data plus an "*"
        data = s.recv(1024)
        print(data)
except KeyboardInterrupt:
    s.close() # close the connection