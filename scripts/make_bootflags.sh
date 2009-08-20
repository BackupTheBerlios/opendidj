#!/bin/bash

# creates an initial Atomic Boot Flags partition JFFS2 image.

set -e

. $PROJECT_PATH/scripts/functions

# make sure all of the environment variables are good
check_vars

# exit if the user is root
check_user

# parse args
set_standard_opts $*

# destination
OUTDIR=./bootflags

if [ "$CLEAN" == "1" ]; then
	rm -rf $OUTDIR
fi

mkdir -p $OUTDIR
echo "RFS0" > $OUTDIR/rootfs
mkfs.jffs2 -n -l -p -e 128KiB -x zlib -x rtime -d $OUTDIR -o $OUTDIR.jffs2

cp $OUTDIR.jffs2 $TFTP_PATH/

exit 0

