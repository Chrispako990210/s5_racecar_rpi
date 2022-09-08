#!/usr/bin/env python

import socket

HOST = '127.0.0.1'
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65432

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as r_client:
    r_client.connect((HOST, PORT))
    #define message format here
    r_client.sendall(b'Hello, world')
    data = r_client.recv(1024)