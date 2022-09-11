#!/usr/bin/env python

import socket
from struct import unpack

HOST = '127.0.0.1' 
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65432

# Messages formats
RPOS_format = ">fff4x"     # > = big-endien, f = float32 (4 octets), x = padding (1 octets) 
OBSF_format = ">I12x"      # I = unint32, x = padding
RBID_format = ">I12x"      # I = unint32, x = padding

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
try:
    while True:
        msg = input("Enter Code :")
        s.send(msg.encode("ASCII")) 
        data = s.recv(1024)
        if not data:
            break
        if msg == "RPOS":
            print(unpack(RPOS_format, data))
        elif msg == "OBSF":
            print(unpack(OBSF_format, data))
        elif msg == "RBID":
            print(unpack(RBID_format, data))
        else:
            print(data.decode("ASCII"))
            
except KeyboardInterrupt:    
    s.close()
