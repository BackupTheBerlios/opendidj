#!/bin/sh
# RDowling 12/10/2007
#
# This script creates a 'otp.bin' image file that is suitable for the
# Matrix Semiconductor programming board (and Windows SW: MatrixUtilityV3)
# to program SanDisk OTP parts.

# Check for usage
if [ ! "$2" -o "$1" != "32" -a "$1" != "64" ]; then
    echo Usage:
    echo '' mkotp.sh SIZE INPUT [OUTPUT]
    echo "Creates a OTP image file from an input zip or tar file, where"
    echo " SIZE is either 32 or 64, indicating the size in MB of the output file,"
    echo " INPUT can be gzipped tar file (.tar.gz), zip (.zip) or leapfrog package (.lfp)"
    echo " OUTPUT if specified overrides default output file of 'output.bin'"
    exit 0
elif [ ! -e $2 ]; then
    echo File $2 does not exist
    exit 1
fi

# Ignore errors for a second
set +e
# Undo any dammage done by last attemp
where=/mnt/ubi-fat
sudo umount $where 2>/dev/null
sudo rmmod ubi 2>/dev/null
sudo rmmod nandsim 2>/dev/null

# Die on errors
set -e

size=$1

# Test for input being tar.gz or lfp
input_file=$2
if ! echo $input_file | grep '.zip$\|.lfp$\|.tar.gz$' >/dev/null; then
	echo "INPUT file must be *.lfp or *.zip or *.tar.gz file"
	exit 1
fi

# Select output file
output_file="otp.bin"
if [ $3 ]; then
    output_file=$3
fi
rm -f $output_file

# Comment out this line if you have installed nandsim.ko with 
# "make modules_install"
local=1

# Set the type of the UBI image: dynamic or static.  
# See ubimkvol and the --type= parameter.
ubi_type=dynamic
# ubi_type=static   # This was an experiment.  It does not seem to work

# Must use ubi name "ubi_Cartridge" or else Lightning will not mount it!
ubi_name="ubi_Cartridge"

# Parameters to nandsim to simulation Toshiba (SanDisk) 32MB OTP part
fib='first_id_byte=0x98'
if [ "$size" = "32" ]; then
	sib='second_id_byte=0x75'	# 32 MB 8 bit 3.3v
	echo "Configured for 32MB part"
else
	sib='second_id_byte=0x76'	# 64 MB 8 bit 3.3v
	echo "Configured for 64MB part"
fi

ecc='ecc_off=1'			# New param to disable ECC

# echo "Making $ubi_type-ubi image $output_file with $ecc"

# This comes from nandsim help.  I guess nandsim wants these
sudo modprobe mtdblock
sudo modprobe mtdchar

# Local installation?
if [ $local ]; then
  # If you have not installed nandsim.ko, you can insmod it this way:
  # There are a number of other modules nandsim wants, so install the original
  # one, then remove it to leave behind all the dependencies.
  # Then bring in nandsim itself, with extra flags
  echo "Using local nandsim.ko"
  sudo modprobe nandsim
  sudo rmmod nandsim
  sudo insmod nandsim.ko $fib $sib $ecc
else
  # You have installed the nandsim.ko module with make modules_install, so 
  # you can modprobe nandsim this way:
  echo "Using installed nandsim.ko"
  sudo modprobe nandsim $fib $sib $ecc
fi

# echo Expect /dev/mtd0
# cat /proc/mtd # This should show /dev/mtd0 # , mtd0r0, mtdblock0
if ! grep mtd0: /proc/mtd > /dev/null; then
    echo "Trouble finding /dev/mtd0 in /proc/mtd"
    exit 1;
fi

# This comes from MTD page
sudo modprobe ubi mtd=0  # This creates /sys/class/ubi/ubi0

# From ~/lightning/LinuxDist/packages/init/etc/init.d/ubi
# echo Expect '"1"'
VC=`cat /sys/class/ubi/ubi0/volumes_count`
if [ "$VC" != "1" ]; then
    echo "Wrong volume count on ubi0 found (1): '$VC'"
    exit 1;
fi

# Don't have ubimkvol...  Had to build it...  This makes volume count=2
# echo ubimkvol name: $ubi_name type: $ubi_type
sudo ./ubimkvol --devn=0 --vol_id=0 --name=$ubi_name -m --type=$ubi_type > /dev/null

# echo Expect '"2"'
VC=`cat /sys/class/ubi/ubi0/volumes_count`
# cat /sys/class/ubi/ubi0/volumes_count # This should return 2
if [ "$VC" != "2" ]; then
    echo "Wrong volume count on ubi0 found (2): '$VC'"
    exit 1;
fi

# echo Expect "/dev/mtd1" with name \"$ubi_name\" to exist in /proc/mtd
X=`cat /proc/mtd` # This should now show /dev/mtd1 with name "ubi_Cartridge"
if ! grep ubi_Cartridge /proc/mtd > /dev/null; then
    echo "Trouble finding ubi_Cartridge in /proc/mtd"
    exit 1;
fi

# Make the FAT file system.  The name "Cartridge" is what Windows will display
sudo mkfs.vfat -n "Cartridge" -F 32 -S 512 /dev/mtdblock1 > /dev/null # make fat 

# Mount it 
sudo mkdir -p $where

sudo mount -t vfat -o noatime /dev/mtdblock1 $where

# Fill it with the image
if echo $input_file | grep '.zip$\|.lfp$' >/dev/null; then
    echo "Unzipping $input_file...  This can take a few seconds..."
    sudo unzip -q -d $where $input_file || \
    	echo "(Ignoring minor unzip errors)"
else
    echo "Untarring $input_file... This can take a few seconds..."
    sudo tar -zxf $input_file -o -C $where || \
	echo "(Ignoring minor tar errors)"
fi

echo `find $where | wc -l` "files extracted"

# Unmount so we can dump raw image (this forces flushing of tar's writes)
sudo umount $where

# Grab the image
sudo nanddump -o /dev/mtd0 -f $output_file 2>/dev/null

# Clean up
sudo rmmod ubi
sudo rmmod nandsim
lsmod > /dev/null # This helps when running the script back-to-back

echo ' '
if [ -e $output_file ]; then
    echo "================="
    echo "SUCCESSFULLY made $output_file"
else
    echo "Trouble making $output_file"
fi
