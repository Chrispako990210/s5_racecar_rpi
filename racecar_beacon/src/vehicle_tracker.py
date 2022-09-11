#!/usr/bin/env python

import socket
from struct import unpack

PORT = 65431

PB_format = ">fffI"        # f = float32, I = unint32

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind(("",PORT))
try:
    while True: # forever
        data, addr = s.recvfrom(1024)
        print(unpack(PB_format, data))
except KeyboardInterrupt:
    s.close() # close the connection