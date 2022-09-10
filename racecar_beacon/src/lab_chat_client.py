#!/usr/bin/env python

import socket

HOST = '127.0.0.1'
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
print ("Enter your msg :")
msg = input()
s.send(msg.encode("UTF_8")) # send same data
data = s.recv(1024) # receive the response
print(data) # print the result
s.close() # close the connection"