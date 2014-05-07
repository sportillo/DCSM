#!/usr/bin/python

"""
A simple UDP transmit script demonstrating how to use
Python to send a UDP packet.

"""

import socket

# Address information of the target (use a broadcast address)
IPADDR = '0.0.0.0'
PORTNUM = 50008

# Initialize the socket (SOCK_DGRAM specifies that this is UDP)

srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
srv.bind((IPADDR, PORTNUM))

# Send the data
while 1:
	data = srv.recv(1024)
	print repr(data)

# Close the socket
s.close()

