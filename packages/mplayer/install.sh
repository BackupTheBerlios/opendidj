#!/bin/bash

# Simply copies a prebuilt ARM version of MPlayer with Brio driver support.
# Building MPlayer from source requires patching some config and driver files.  

RELEASES_PATH=$PROJECT_PATH/releases

set -e

. $PROJECT_PATH/scripts/functions

# make sure all of the environment variables are good
check_vars

# exit if the user is root
check_user

# parse args
set_standard_opts $*

pushd $PROJECT_PATH/packages/mplayer
cp $RELEASES_PATH/mplayer-arm-bin-004.tar.gz .
tar -xzvf mplayer-arm-bin-004.tar.gz
cp mplayer $ROOTFS_PATH/usr/bin
popd

exit 0
