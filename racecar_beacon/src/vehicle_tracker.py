#!/usr/bin/env python

import socket
from struct import *

HOST = '127.0.0.1'  # ros_monitor adress
# This process should listen to a different port than the RemoteRequest client.
PORT = 65431

format = ">fffI"

# Affiche l'adresse IP d'ou provient le signal de position (So adresse IP du racecar)

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as vt: # AF_INET = IPv4, SOCK_DGRAM = UDP
    vt.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # Enable broadcasting mode
    vt.bind(("", PORT))
    print("Vehicule tracker socket created")

    while True:
        data = vt.recv(16)
        print(unpack(format,data))