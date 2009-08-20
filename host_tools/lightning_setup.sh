#!/bin/bash
# Load images onto development board
# Typical command: 'lightning_setup.sh <CPU> <BOARD>'

# default values, override with command line, or env settings
DEFAULT_BOARD=LF
DEFAULT_CPU=MP2530F
DEFAULT_ROOTFS_PATH=/home/lfu/nfsroot
DEFAULT_SERIAL_PORT=/dev/ttyS0
DEFAULT_TFTP_PATH=/tftpboot
DEFAULT_VERSION=0.7.2-1659

# parse command line
ARGS=`getopt -o b:c:hr:s:t:v \
--long board:,cpu:,help,rootfs_path:,serial_port:,tftp_path:,version -- "$@"`

eval set -- "$ARGS"

while true ; do
	case "${1}" in
		-b|--board )       BOARD=${2}       ; shift 2 ;;
		-c|--cpu )         CPU=${2}         ; shift 2 ;;
		-h|--help )        HELP=y           ; shift   ;;
		-r|--rootfs_path ) ROOTFS_PATH=${2} ; shift 2 ;;
		-s|--serial_port ) SERIAL_PORT=${2} ; shift 2 ;;
		-t|--tftp_path )   TFTP_PATH=${2}   ; shift 2 ;;
		-v|--version )     VERSION=y        ; shift   ;;
		--)                shift            ; break   ;;

		*)	echo "Internal error!"           ;
			echo "Press Enter to close..."
			read
	       		exit 1 ;;
	esac
done

# If show help, display info and exit
if [ "X${HELP}" == "Xy" ]
then
	echo
	echo "${0} Flash software on board, options are:"
	echo "  -b, --board        board type LF (LeapFrog) or ME (MagicEyes), \
default is '${DEFAULT_BOARD}'"
	echo "  -c, --cpu          cpu type of LF1000 or MP2530F, \
default is '${DEFAULT_CPU}'"
	echo "  -h, --help         this list"
	echo "  -r, --rootfs_path  rootfs directory, \
default is '${DEFAULT_ROOTFS_PATH}'"
	echo "  -s, --serial_port  serial port, \
default is '${DEFAULT_SERIAL_PORT}'"
	echo "  -t, --tftp_path    tftp directory, \
default is '${DEFAULT_TFTP_PATH}'"
	echo "  -v, --version      software version, \
default is '${DEFAULT_VERSION}'"
	echo
	echo "Press Enter to close..."
	read
	exit 0
fi

# validate BOARD setting
if [ "X${BOARD}" == "X" ]
then
	BOARD=${DEFAULT_BOARD}
	echo "BOARD not set, using default '${BOARD}'"
fi

if [ ${BOARD} != "LF" -a ${BOARD} != "ME" ]
then
	echo "Expected BOARD to be LF or ME, instead of '${BOARD}'."
	echo "Press Enter to close..."
	read
	exit 1
fi

# validate CPU setting
if [ "X${CPU}" == "X" ]
then
	CPU=${DEFAULT_CPU}
	echo "CPU not set, using default '${CPU}'"
fi

if [ ${CPU} != "LF1000" -a ${CPU} != "MP2530F" ]
then
	echo "Expected CPU to be LF1000 or MP2530F, instead of '${CPU}'."
	echo "Press Enter to close..."
	read
	exit 1
fi

# validate ROOTFS_PATH setting
if [ "X${ROOTFS_PATH}" == "X" ]
then
	ROOTFS_PATH=${DEFAULT_ROOTFS_PATH}
	echo "ROOTFS_PATH not set, using default '${ROOTFS_PATH}'"
fi

if [ ! -d ${ROOTFS_PATH} ]
then
	echo "ROOTFS_PATH directory '${ROOTFS_PATH}' does not exist."
	echo "Press Enter to close..."
	read
	exit 1
fi

# validate ROOTFS_PATH/dev setting
if [ ! -d ${ROOTFS_PATH}/dev ]
then
	echo "ROOTFS_PATH device directory '${ROOTFS_PATH}/dev' does not exist."
	echo "Press Enter to close..."
	read
	exit 1
fi

# validate SERIAL_PORT setting
if [ "X${SERIAL_PORT}" == "X" ]
then
	SERIAL_PORT=${DEFAULT_SERIAL_PORT}
	echo "SERIAL_PORT not set, using default '${SERIAL_PORT}'"
fi

# ensure device exists
if [ ! -e ${SERIAL_PORT} ]
then
	echo "SERIAL_PORT '${SERIAL_PORT}' does not exist."
	echo "Press Enter to close..."
	read
	exit 1
fi

# ensure it's a serial port, or at least a character device
if [ ! -c ${SERIAL_PORT} ]
then
	echo "SERIAL_PORT '${SERIAL_PORT}' is not a character device."
	echo "Press Enter to close..."
	read
	exit 1
fi

# validate TFTP_PATH setting
if [ "X${TFTP_PATH}" == "X" ]
then
	TFTP_PATH=${DEFAULT_TFTP_PATH}
	echo "TFTP_PATH not set, using default '${TFTP_PATH}'"
fi

if [ ! -d $TFTP_PATH ]
then
	echo "TFTP_PATH directory '${TFTP_PATH}' does not exist."
	echo "Press Enter to close..."
	read
	exit 1
fi

# validate software VERSION setting
if [ "X${VERSION}" == "X" ]
then
	VERSION=${DEFAULT_VERSION}
	echo "VERSION not set, using default '${VERSION}'"
fi

# set CPU specific variables
if [ "$CPU" == "LF1000" ]
then
	BOOT=UARTBOOT.bin
	BOOTSTRAP=lf1000_bootstrap.py
	CONSOLE=ttyS0
elif [ "$CPU" == "MP2530F" ]
then
	BOOT=boot-u.nb0
	BOOTSTRAP=mp2530_bootstrap.py
	CONSOLE=ttyS3
else
	echo "Unexpected CPU value of '$CPU'."
	echo "Press Enter to close..."
	read
	exit 1
fi

# check for files
for FILENAME in ${TFTP_PATH}/${BOOT} \
	        ${TFTP_PATH}/uniboot-${VERSION}-${CPU}.bin \
                ${TFTP_PATH}/u-boot-${VERSION}-${CPU}.bin \
		${TFTP_PATH}/zImage-${VERSION}-${CPU}
do
	if [ ! -e ${FILENAME} ]
	then
		echo "Missing file '$FILENAME'."
		echo "Press Enter to close..."
		read
		exit 1
	fi
done

# Load bootstrap onto board
${BOOTSTRAP} ${SERIAL_PORT} ${TFTP_PATH}/${BOOT} ${TFTP_PATH}/u-boot-${VERSION}-${CPU}.bin

if [ $? -ne 0 ]
then
	echo "Error loading bootstrap onto board."
	echo "Press Enter to close..."
	read
	exit 1
fi

# flash images onto board
lightning_install.py ${SERIAL_PORT} -e \
${TFTP_PATH}/uniboot-${VERSION}-${CPU}.bin:0 \
${TFTP_PATH}/u-boot-${VERSION}-${CPU}.bin:80000 \
${TFTP_PATH}/zImage-${VERSION}-${CPU}:100000

if [ $? -ne 0 ]
then
	echo "Error flashing images onto board."
	echo "Press Enter to close..."
	read
	exit 1
fi

# setup console in shared nfsroot image for specific board
pushd ${ROOTFS_PATH}/dev
ln -sf ${CONSOLE} console
popd

echo "Press Enter to close..."
read
