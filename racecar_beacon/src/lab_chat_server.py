#!/usr/bin/env python

import socket

HOST = '10.0.1.21'
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
conn, addr = s.accept()
print("connceted: ", conn, " adress: ", addr)
while True:
    data = conn.recv(1024)
    if not data:
        break
    print(data)
    msg = input(">")
    conn.send(msg.encode("utf-8"))
conn.close()

