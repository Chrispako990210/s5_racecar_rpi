#!/usr/bin/env python

import socket

HOST = '127.0.0.1'  # HOST = l'autre machine, donc pour le server l'host c'est le client et pour le client l'host c'est le server
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))

s.listen(1)
(conn, addr) = s.accept() # returns new socket and addr. client
while True: # forever
    data = conn.recv(1024) # receive data from client
    if not data:
        break # stop if client stopped
    print(data)
    conn.send(data + "*".encode("UTF_8")) # return sent data plus an "*"
conn.close() # close the connection