#!/bin/bash

set -e

. $PROJECT_PATH/scripts/functions

# make sure all of the environment variables are good
check_vars

set_standard_opts $*

pushd $PROJECT_PATH/u-boot-1.1.6-lf1000/

if [ "$CLEAN" == "1" ]; then
	rm -f board/lf1000/autoconf.h
	rm -f include/configs/lf1000_mach.h
        make clean
fi

# XXX: this is a u-boot specific cross compiler, we will hopefully make it go
#      away eventually.  For now, leaving it here and not setting it in the
#      overall build system functions.
if [ "X$UBOOT_CROSS_COMPILE" == "X" ]; then
	export UBOOT_CROSS_COMPILE=/opt/crosstool/gcc-3.4.5-glibc-2.3.6/arm-softfloat-linux-gnu/bin/arm-softfloat-linux-gnu-
	echo "UBOOT_CROSS_COMPILE not set, using default $UBOOT_CROSS_COMPILE"
fi

# grab the kernel config
LINUX_CONFIG=include/linux/autoconf.h
if [ -e $KERNELDIR/$LINUX_CONFIG ]; then
	cp $KERNELDIR/$LINUX_CONFIG board/lf1000
	cat $KERNELDIR/$LINUX_CONFIG | grep CONFIG_MACH > include/configs/lf1000_mach.h
	cat $KERNELDIR/$LINUX_CONFIG | grep CONFIG_NAND_LF1000 >> include/configs/lf1000_mach.h
else
	echo "Error: $KERNELDIR/$LINUX_CONFIG not found, you must configure and build the kernel before running this script."
	echo "Or perhaps your KERNELDIR environment var is not set properly?"
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

# build u-boot
rm -f board/lf1000/.depend
KERNELDIR=$KERNELDIR CROSS_COMPILE=$UBOOT_CROSS_COMPILE make lf1000_config
KERNELDIR=$KERNELDIR CROSS_COMPILE=$UBOOT_CROSS_COMPILE make all
cp u-boot.bin $TFTP_PATH

$PROJECT_PATH/host_tools/mkimage -T script -A arm -C none -n 'Boot Menu' -d uboot_init.txt $TFTP_PATH/uboot_init.img
$PROJECT_PATH/host_tools/mkimage -T script -A arm -C none -n 'Boot Menu' -d uboot_init_mp2530.txt $TFTP_PATH/uboot_init_mp2530.img

popd
