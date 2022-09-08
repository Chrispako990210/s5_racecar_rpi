#!/usr/bin/env python

import socket

HOST = '192.168.137.10' #'127.0.0.1'
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

s.send('Hello, world') # send same data
data = s.recv(1024) # receive the response
print(data) # print the result
s.close() # close the connection