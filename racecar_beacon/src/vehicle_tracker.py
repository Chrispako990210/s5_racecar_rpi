#!/usr/bin/env python

import socket

HOST = '127.0.0.1'
# This process should listen to a different port than the RemoteRequest client.
PORT = 65431

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
try:
    while True: # forever
        data = s.recv(1024)
        print(data)
except KeyboardInterrupt:
    s.close() # close the connection