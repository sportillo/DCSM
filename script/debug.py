#!/usr/bin/python

"""
A simple UDP transmit script demonstrating how to use
Python to send a UDP packet.

"""

import socket
import time
import datetime
from struct import *

ts = time.time()
st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d--%H-%M-%S')
# Address information of the target (use a broadcast address)
IPADDR = '0.0.0.0'
PORTNUM = 50008
log_file = open('log_'+st+'.txt', 'w')

# Initialize the socket (SOCK_DGRAM specifies that this is UDP)

srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
srv.bind((IPADDR, PORTNUM))

# Send the data
while 1:
	buf = bytearray(320)
	view = memoryview(buf)
	nbytes = srv.recv_into(view, 320)

	for i in range(0,8):
		s = unpack_from('dddd',buf,i*32)
		log_file.write(str(s[0]) + " " + str(s[1]) + " " + str(s[2]) + " " +str(s[3]) +"\n")
		print ["%0.2f" % j for j in s]


# Close the socket
srv.close()
out_file.close()


