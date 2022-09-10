#!/usr/bin/env python

import socket

HOST = '127.0.0.1'
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
try:
    while True:
        msg = input(">")
        s.send(msg.encode("ASCII")) 
        data = s.recv(1024)
        if not data:
            break
        print(data.decode("ASCII"))
except KeyboardInterrupt:    
    s.close()

