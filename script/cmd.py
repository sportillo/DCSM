#!/usr/bin/python

"""
A simple UDP transmit script demonstrating how to use
Python to send a UDP packet.

"""

import socket

# Address information of the target (use a broadcast address)
IPADDR = '192.168.2.255'
PORTNUM = 50007

# Initialize the socket (SOCK_DGRAM specifies that this is UDP)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# Connect the socket
s.connect((IPADDR, PORTNUM))

# Send the data
while 1:
	PACKETDATA = raw_input()
	s.send(PACKETDATA)

# Close the socket
s.close()

