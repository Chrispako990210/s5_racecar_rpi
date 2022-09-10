#!/usr/bin/env python

import socket

HOST = '127.0.0.1'
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
conn, addr = s.accept()
print("connceted,  adress: ", addr)
try:
    while True:
        data = conn.recv(1024)
        if not data:
            break
        print(data.decode("ASCII"))
        #msg = input(">")
        #conn.send(msg.encode("UTF-8"))
        if data.decode("ASCII") == 'abc':
            reply = '123'
            conn.send(reply.encode("ASCII"))
        else:
            reply2 = "jsp"
            conn.send(reply2.encode("ASCII"))
except KeyboardInterrupt:
    conn.close()

