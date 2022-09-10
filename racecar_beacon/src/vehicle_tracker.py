#!/usr/bin/env python3

import socket


HOST = '127.0.0.1'  # ros_monitor adress
# This process should listen to a different port than the RemoteRequest client.
PORT = 65431

# Affiche l'adresse IP d'ou provient le signal de position (So adresse IP du racecar)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect((HOST, PORT))
vt_format = ">fffI"                # > = big-endien, f = float32 (4 octets), x = padding (1 octets), ici 16 Bytes
try:
    while True: 
        data = s.recv(16)
        if not data:
            break
        print(data.decode(format))
except KeyboardInterrupt:    
    s.close()