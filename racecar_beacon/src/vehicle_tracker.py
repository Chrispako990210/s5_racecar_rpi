#!/usr/bin/env python3

import socket
from struct import unpack

HOST = '127.0.0.1'  # ros_monitor adress
# This process should listen to a different port than the RemoteRequest client.
PORT = 65431

format = ">fffI"

# Affiche l'adresse IP d'ou provient le signal de position (So adresse IP du racecar)
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as pb_socket:  # AF_INET = IPv4, SOCK_DGRAM = UDP
    pb_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # Enable broadcasting mode
    pb_socket.bind(("", PORT))
    vt_format = ">fffI"                # > = big-endien, f = float32 (4 octets), x = padding (1 octets), ici 16 Bytes
    try:
        while True: 
            data = pb_socket.recv(16)
            if not data:
                break
            print(unpack(vt_format, data))
    except KeyboardInterrupt:    
        pb_socket.close()