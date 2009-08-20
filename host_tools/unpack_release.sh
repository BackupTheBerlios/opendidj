#!/bin/bash
#
# release 'unpacking' convenience script
set -e

BASEDIR=../
if [ "$1" != "" ]; then
	BASEDIR=$1
fi

# check some variables
if [ "X$TFTP_PATH" == "X" ]; then
	echo "Error: your TFTP_PATH is not set."
	exit 1;
fi
if [ "X$ROOTFS_PATH" == "X" ]; then
	echo "Error: your ROOTFS_PATH is not set."
	exit 1;
fi

# get board info
echo "Which board are you using?"
echo "	1. LF1000 Form Factor Board"
echo "	2. LF1000 Development Board"
#echo "	3. Leapfrog MP2530F Development Board"
while read -p "Choose [1-2] " ans; do
	if [ "$ans" == "1" ]; then
		BOARD=LF_LF1000
		break;
	elif [ "$ans" == "2" ]; then
		BOARD=ME_LF1000
		break;
#	elif [ "$ans" == "3" ]; then
#		BOARD=LF_MP2530F
#		break;
	fi
done

# do the unpack
if [ -e $ROOTFS_PATH ]; then
	while read -p "Safe to delete $ROOTFS_PATH? [Yn] " ans; do
		if [ "$ans" == "y" -o "$ans" == "Y" -o "$ans" == "" ]; then
			sudo rm -rf $ROOTFS_PATH
			break;
		elif [ "$ans" == "n" -o "$ans" == "N" ]; then
			exit 0
		else
			echo "Invalid answer $ans"
		fi
	done
fi
mkdir -p $ROOTFS_PATH
sudo cp -R $BASEDIR/nfsroot-*-$BOARD/* $ROOTFS_PATH

# unpack the lfps and put the images in the proper place
TMPDIR=/tmp/lightning_unpack
rm -rf $TMPDIR
mkdir -p $TMPDIR

$BASEDIR/host_tools/lfpkg -a install -b $TMPDIR -d . $BASEDIR/packages/firmware-$BOARD-*.lfp
$BASEDIR/host_tools/lfpkg -a install -b $TMPDIR -d . $BASEDIR/packages/bootstrap-$BOARD-*.lfp

cp $TMPDIR/firmware-$BOARD/kernel.bin $TFTP_PATH/kernel.bin
cp $TMPDIR/firmware-$BOARD/erootfs.jffs2 $TFTP_PATH/erootfs.jffs2
cp $TMPDIR/bootstrap-$BOARD/lightning-boot.bin $TFTP_PATH/lightning-boot.bin


# copying the ATAP files into the correct place for automated ATAP cartidge creation
if [ -d $BASEDIR/mfg-cart/ATAP ] ; then
	sudo mkdir -p $ROOTFS_PATH/payload
	sudo mkdir -p $ROOTFS_PATH/payload/ATAP
	sudo mkdir -p $ROOTFS_PATH/payload/ATAP/FW_packages
	sudo cp -Rf $BASEDIR/mfg-cart/* $ROOTFS_PATH/payload 
	sudo cp -f $BASEDIR/packages/*.lfp $ROOTFS_PATH/payload/ATAP/FW_packages 
fi

echo "done"
