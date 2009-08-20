#!/usr/bin/env python
#
# automated LF1000 Board Bringup and testing
# 
# Andrey Yurovsky <andrey@cozybit.com
###############################################################################

import sys
import os
import time
import serial
import re

try:
	tftp = os.environ['TFTP_PATH']
except KeyError:
	print "Error: TFTP_PATH not set.  Please set it."
	sys.exit(1)

MAP_FILE='flash.map'

# default paths (overridden by user)
UNIBOOT_PATH="/tftpboot/uniboot.bin"
UBOOT_PATH="/tftpboot/u-boot.bin"
KERNEL_PATH="/tftpboot/zImage"

# basic settings
PORT="/dev/ttyS0" #default
RATE=115200
BLOCK_SIZE=0x800
U_BOOT_RETRIES=20

re_transfer = re.compile(r'Bytes transferred = \d* \((\w*) hex\)')
re_crc32 = re.compile(r'CRC32 for 02000000 ... \w{8} ==> (\w*)\r\n')
re_write = re.compile(r'(\d*) bytes written: OK')
re_read = re.compile(r'(\d*) bytes read: OK')

def path_to_file(p):
	return (p.split('/')).pop()

def eat(s, lines):
	for i in range(0,lines):
		s.readline()

# see if port is busy, 1 for busy
def port_is_busy(port):
	return(not os.system('lsof -w %s > /dev/null' % port))

def clear_mem(s, size):
	s.write("mw.b 02000000 0 %X\n" % size)
	eat(s, 2)

def nand_read(s, src, size):
	s.write("nand read 02000000 %X %X\n" % (src, size))
	eat(s, 3)
	res = s.readline()
	while res == '':
		res = s.readline()
	m = re_read.match(res.strip())
	if not m:
		return False
	size = m.groups()[0]
	s.readline()
	return int(size)

def nand_write(s, dest, size):
	s.write("nand write 02000000 %X %X\n" % (dest, size))
	eat(s, 3)
	res = s.readline()
	while res == '':
		res = s.readline()
	m = re_write.match(res.strip())
	if not m:
		return False
	size = m.groups()[0]
	s.readline()
	return int(size)

def nand_erase_area(s, dest, size):
	s.write("nand erase %X %X\n" % (dest, size))
	eat(s, 3)
	res = s.readline().strip()
	while res.startswith('Erasing at') or res == '':
		res = s.readline().strip()
	if res != 'OK':
		return False
	return True

def crc32(s, size):
	s.write("crc32 02000000 %X\n" % size)
	ser.readline()
	res = s.readline()
	while res == '':
		res = s.readline()
	m = re_crc32.match(res)
	if not m:
		return False
	crc = m.groups()[0]
	ser.readline()
	return int(crc, 16)

def transfer(s, file):
	s.write("tftp 02000000 %s\n" % file)
	eat(s, 4)
	res = s.readline()
	if not res.startswith("Loading:"):
		return False
	res = s.readline()
	while res.strip().startswith('#'):
		res = s.readline()
	if res.strip() != "done":
		return False
	res = s.readline() # get bytes transmitted
	m = re_transfer.match(res)
	if not m:
		return False
	size = m.groups()[0]
	eat(s, 3)
	return int(size, 16)

def write_file(s, file, dest, erase_first=False):
	size = transfer(ser, file)
	if size == False:
		print "FAIL: transfer", file
		return False
	print "PASS: transfer", file

	if size > BLOCK_SIZE:
		rem = size % BLOCK_SIZE
		if rem == 0:
			p_size = size
		else:
			p_size = size - rem + BLOCK_SIZE
	else:
		p_size = BLOCK_SIZE

	crc = crc32(ser, size)
	if crc == False:
		print "FAIL: crc32", file
		return False
	print "PASS: crc32", file

	if erase_first == True:
		erase = nand_erase_area(ser, dest, p_size)
		if erase == False:
			print "FAIL: erase area for", file
			return False
		print "PASS: erase area for", file

	w_size = nand_write(ser, dest, p_size)
	if w_size == False or w_size != p_size:
		print "FAIL: write", file
		return False
	print "PASS: write", file

	clear_mem(ser, size)

	r_size = nand_read(ser, dest, p_size)
	if r_size == False or r_size != p_size:
		print "FAIL: read back", file
		return False
	print "PASS: read back", file

	r_crc = crc32(ser, size)
	if r_crc == False:
		print "FAIL: crc32", file
		return False
	print "PASS: crc32", file

	if r_crc != crc:
		print "FAIL: crc doesn't match for", file
		return False
	print "PASS: crc32 matches for", file

	return True

def run(images, erase_all=False):
	ser.flushInput()
	ser.flushOutput()
	time.sleep(3)

	#############################
	# make sure u-boot is there #
	#############################

	ser.write(" \n")
	time.sleep(1)
	res = ser.readline()
	i = 0
	while res.strip() != "LF1000 #" and i < U_BOOT_RETRIES:
		#print "got:",res
		res = ser.readline()
		i = i + 1
	if i >= U_BOOT_RETRIES:
		print "FAIL: u-boot communication"
		return False
	print "PASS: u-boot communication"

	##########################
	# test Ethernet via ping #
	##########################

	ser.write("ping 192.168.0.113\n")
	ser.readline()
	res = ser.readline()
	if res.strip() == "host 192.168.0.113 is alive":
		print "PASS: Ethernet"
	else:
		print "FAIL: Ethernet"
		return False
	ser.readline() # eat prompt

	##############
	# nand erase #
	##############

	if erase_all == True:
		ser.write("nand erase\n")
		eat(ser, 3)
		res = ser.readline()
		res = res.strip()
		bad = 0
		while res.startswith('Skipping bad block') or \
		      res.startswith('Erasing at'):
			res = ser.readline()
			res = res.strip()
		if res.strip() == "OK":
			print "PASS: NAND erase"
		else:
			print "FAIL: NAND erase" 
			return False
		ser.readline() # eat prompt

	##############################
	# write images to NAND Flash # 
	##############################

	for image in images:
		res = write_file(ser, image[0], image[1], not erase_all)
		if not res:
			return False

	########
	# done #
	########

	return True

if __name__ == '__main__':
	files = []
	port = '/dev/ttyS0'

	# print the help if needed
	if len(sys.argv) >= 2 and sys.argv[1] == '-h':
		print "usage %s <port> [-e] <file1:addr1> [file2:addr2] ..." \
		      "\n\n" \
		      "\toptions:  -e Erase entire NAND Flash first.\n" % \
			sys.argv[0]
		sys.exit(0)

	# get the serial port
	if len(sys.argv) < 2:
		print "assuming serial port /dev/ttyS0"
	else:
		port = sys.argv[1]

	# see if port is busy
	if port_is_busy(port):
		sys.stderr.write("Serial port '%s' is in use\n" % port)
		sys.exit(1)

	# use map file, if there is one
	if os.path.exists(MAP_FILE):
		print "using memory map from ./%s" % MAP_FILE
		try:
			f = open(MAP_FILE)
		except IOError:
			print "Error: failed to open %s" % MAP_FILE
			sys.exit(1)
		i = 0
		lines = f.readlines()
		for line in lines:
			file = line.split(':')
			# make sure format is correct and file exists
			if len(file) != 2:
				print "skipping line %d, invalid format" % (i-1)
			elif not os.path.exists(tftp+'/'+file[0]):
				print "skipping line %d, file not found" % (i-1)
			else:
				files.append([file[0],int(file[1], 16)])
			i+=1

	# parse options: 'erase all' and any manually-specified files
	i = 2
	erase_all = False
	while i < len(sys.argv):
		if sys.argv[i] == '-e':
			erase_all = True
			i += 1
			continue
		file = sys.argv[i].split(':')
		# make sure format is correct
		if len(file) != 2:
			print "skipping file %d, invalid format" % (i-1)
			i += 1
			continue
		# make sure path is valid
		if os.path.exists(tftp+'/'+file[0]):
			name = file[0]
		else:
			print "skipping file %d, invalid path" % (i-1)
			i += 1
			continue
		# make sure address is at least a number in hex
		try:
			addr = int(file[1],16)
		except ValueError:
			print "skipping file %d, invalid format" % (i-1)
			i += 1
			continue
		# save the file info
		files.append([name,int(addr)])
		i += 1

	if len(files) == 0:
		print "no files to install, exiting..."
		sys.exit(1)

	try:
		ser = serial.Serial(port=port, baudrate=RATE, bytesize=8,\
			parity='N', stopbits=1, timeout=1,\
			xonxoff=0, rtscts=0, writeTimeout=None,\
			dsrdtr=None)
	except:
		sys.stderr.write("Could not open port\n")
		sys.exit(1)

	try:
		result = run(files, erase_all)
	except KeyboardInterrupt:
		print "^C received, exiting..."
		ser.close()
		sys.exit()

	if result:
		print "board programming passed"
	else:
		print "board programming failed"
	ser.close()
