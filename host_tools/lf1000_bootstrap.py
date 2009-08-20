#!/usr/bin/env python

import sys
import os
import serial
import time

# see if port is busy, 1 for busy
def port_is_busy(port):
	return(not os.system('lsof -w %s > /dev/null' % port))

# send_file -- transmit a file to the LF1000
#
# file  - path to file to transmit
# port  - serial port to use
# rate  - baud rate to use
# delay - delay (in seconds, floating point) between bytes.
# mode  - 0 for hardware bootstrap, 1 for bootloader.  In bootloader mode, the
#         file size is written before the file data.
def send_file(file, port, rate, delay=0, mode=0):
	# set up the serial port
	try:
		ser = serial.Serial(port=port, baudrate=rate, bytesize=8,\
				parity='N', stopbits=1, timeout=None,\
				xonxoff=0, rtscts=0, writeTimeout=None,\
				dsrdtr=None)
	except:
		sys.stderr.write("Could not open port\n")
		return -1

	ser.flushOutput()

	# get the file
	try:
		h = open(file, "r")
	except:
		ser.close()
		sys.stderr.write("Could not open \"%s\"\n" % file)
		return -1

	# read the data
	data = h.read()
	length = len(data)
	chunk = length/79
	i = 0

	print "sending \"%s\", size=0x%X" % (file, length)

	# send file size (in 32-bit little endian representation), if needed
	if mode != 0:
		print "selecting option 1..."
		time.sleep(1)
		ser.write('1')
		time.sleep(1)

		size = length+4
		ser.write(chr(((size>>0)  & 0xFF)))
		ser.write(chr(((size>>8)  & 0xFF)))
		ser.write(chr(((size>>16) & 0xFF)))
		ser.write(chr(((size>>24) & 0xFF)))

	# send the file
	for di in range(0, length):
		ser.write(data[di])
		if delay > 0:
			time.sleep(delay)
		i += 1
		if i >= chunk:
			sys.stdout.write('#')
			sys.stdout.flush()
			i = 0

	# send padding, if needed
	if mode == 0 and length < 0x4000:
		length = 0x4000 - length
		chunk = (length-1)/79
		print "\nsending 0x%X bytes of padding..." % (length)
		i = 0
		for di in range(0, length):
			ser.write('0') # junk/padding
			if delay > 0:
				time.sleep(delay)
			i += 1
			if i >= chunk:
				sys.stdout.write('#')
				sys.stdout.flush()
				i = 0

	if mode != 0:
		print "\nselecting option 2..."
		time.sleep(1)
		ser.write('2')

	h.close()
	ser.close()
	print "\ndone"

###########################################
# send bootstrap and bootloader to LF1000 #
###########################################

if __name__ == '__main__':
	if len(sys.argv) < 3:
		print "usage: %s <port> <bootstrap> <bootloader>" % sys.argv[0]
		sys.exit(0)

	# see if port is busy
	if port_is_busy(sys.argv[1]):
		sys.stderr.write("Serial port '%s' is in use\n" % sys.argv[1])
		sys.exit(-1)
	
	try:
		# bootstrap, a small delay is required between bytes
		send_file(sys.argv[2], sys.argv[1], 19200, delay=0.001)

		# bootloader
		if len(sys.argv) >= 4:
			time.sleep(5)
			send_file(sys.argv[3], sys.argv[1], 115200, mode=1)
	
	except KeyboardInterrupt:
		print "\n^C received, exiting..."
		sys.exit(0)
