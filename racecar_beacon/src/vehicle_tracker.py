#!/usr/bin/env python3

import socket


HOST = '127.0.0.1'  # ros_monitor adress
# This process should listen to a different port than the RemoteRequest client.
PORT = 65431

# Affiche l'adresse IP d'ou provient le signal de position (So adresse IP du racecar)
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as pb_socket:  # AF_INET = IPv4, SOCK_DGRAM = UDP
    
    pb_socket.connect((HOST, PORT))
    vt_format = ">fffI"                # > = big-endien, f = float32 (4 octets), x = padding (1 octets), ici 16 Bytes
    try:
        while True: 
            data = pb_socket.recv(16)
            if not data:
                break
            print(data.decode(format))
    except KeyboardInterrupt:    
        pb_socket.close()