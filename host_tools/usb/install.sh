#!/bin/bash

set -e

. $PROJECT_PATH/scripts/functions

# make sure all of the environment variables are good
check_vars

# parse args
set_standard_opts $*

pushd $PROJECT_PATH/host_tools/usb

gcc -o testusb testusb.c -lpthread

cp testusb $PREFIX/bin/
cp testusb_all $PREFIX/bin/

popd

exit 0
