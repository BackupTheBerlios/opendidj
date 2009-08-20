#!/bin/bash

# Build MPlayer from source
MPLAYER_BIN_SRC=MPlayer-1.0rc1.tar.bz2

# Patches needed for
# (1) adding Brio video output drivers
# (2) avoiding includes from '/usr/include'
# (3) resolving __strdup and _strtol references

set -e

. $PROJECT_PATH/scripts/functions

# make sure all of the environment variables are good
check_vars

# exit if the user is root
check_user

# parse args
set_standard_opts $*

pushd $PROJECT_PATH/packages/mplayer/

if [ ! -e $MPLAYER_BIN_SRC ]; then
	wget http://www3.mplayerhq.hu/MPlayer/releases/$MPLAYER_BIN_SRC
fi

MPLAYER_BIN_DIR=`echo "$MPLAYER_BIN_SRC" | cut -d '.' -f -2`
echo $MPLAYER_BIN_DIR

if [ "$CLEAN" == "1" -o ! -e $MPLAYER_BIN_DIR ]; then
	rm -rf $MPLAYER_BIN_DIR
	tar -xjf $MPLAYER_BIN_SRC
fi

# build and copy binary to rootfs
pushd $MPLAYER_BIN_DIR
./configure --enable-cross-compile --cc=arm-linux-gcc --host-cc=gcc --target=arm-linux --enable-armv5te --disable-iwmmxt --enable-theora --disable-tremor-internal --disable-tremor-low --enable-tremor-external --disable-libvorbis --disable-alsa --disable-mp3lib --disable-liba52 --disable-libmpeg2 --prefix=$ROOTFS_PATH/usr --libdir=$ROOTFS_PATH/usr/lib --with-extraincdir=$ROOTFS_PATH/usr/local/include --with-extralibdir=$ROOTFS_PATH/usr/local/lib
make
cp mplayer $ROOTFS_PATH/usr/bin/
popd

popd

exit 0
