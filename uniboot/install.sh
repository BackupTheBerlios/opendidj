#!/bin/bash

# This script builds a uniboot image.

set -e

. $PROJECT_PATH/scripts/functions

# make sure all of the environment variables are good
check_vars

set_standard_opts $*

pushd $PROJECT_PATH/uniboot/

if [ "$CLEAN" == "1" ]; then
	make clean
fi

# grab the kernel config
LINUX_CONFIG=include/linux/autoconf.h
if [ -e $KERNELDIR/$LINUX_CONFIG ]; then
	cp $KERNELDIR/$LINUX_CONFIG ./
else
	echo "Error: $KERNELDIR/$LINUX_CONFIG not found, you must configure and build the kernel before running this script."
	exit 1
fi

# grab the machine types
MACH_TYPES=include/asm-arm/mach-types.h
if [ -e $KERNELDIR/$MACH_TYPES ]; then
	cp $KERNELDIR/$MACH_TYPES ./
else
	echo "Error: $KERNELDIR/$MACH_TYPES not found, you must configure and build the kernel for ARM before running this script."
	exit 1
fi

KERNELDIR=$KERNELDIR CROSS_COMPILE=$CROSS_COMPILE make

cp uniboot.bin $TFTP_PATH

popd
