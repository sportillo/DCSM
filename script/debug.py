#!/usr/bin/python

"""
A simple UDP transmit script demonstrating how to use
Python to send a UDP packet.

"""

import socket

# Address information of the target (use a broadcast address)
IPADDR = '192.168.1.255'
PORTNUM = 50007

# Data content of the UDP packet
PACKETDATA = ''

# Initialize the socket (SOCK_DGRAM specifies that this is UDP)

srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
srv.bind(('0.0.0.0', 50007))

# Connect the socket

# Send the data
while 1:
	data = srv.recv(1024)
	print repr(data)

# Close the socket
s.close()

