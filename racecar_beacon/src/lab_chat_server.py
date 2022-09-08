#!/usr/bin/env python

import socket

HOST = '192.168.137.10' #'127.0.0.1'
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))

s.listen(1)
(conn, addr) = s.accept() # returns new socket and addr. client
while True: # forever
    data = conn.recv(1024) # receive data from client
    if not data: 
        break # stop if client stopped
    #conn.send(raw_input("> ")) # send data to client
    conn.raw_input("> ") # send data to client
    #conn.send(str(data)+"*") # return sent data plus an "*"
conn.close() # close the connection