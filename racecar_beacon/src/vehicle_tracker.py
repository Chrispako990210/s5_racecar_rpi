#!/usr/bin/env python

import socket


HOST = '127.0.0.1'  # ros_monitor adress
# This process should listen to a different port than the RemoteRequest client.
PORT = 65431

# Affiche l'adresse IP d'ou provient le signal de position (So adresse IP du racecar)

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as vt: # AF_INET = IPv4, SOCK_DGRAM = UDP
    vt.sendto(things_to_send, (HOST, PORT))
    data = vt.recv(1024)
    print(data)