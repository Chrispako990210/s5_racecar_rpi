#!/usr/bin/env python

import socket

HOST = '192.168.137.109'
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
s.listen(1)
conn, addr = s.accept()
while True:
    data = conn.recv(1024)
    if not data:
        break
    print(data)
    msg = input(">")
    conn.send(str(msg))
conn.close()

