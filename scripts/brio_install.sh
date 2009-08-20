#!/bin/bash

echo "WARNING! The $0 Brio script is deprecated and does not work!"
exit 1

set -e

. $PROJECT_PATH/scripts/functions

# make sure all of the environment variables are good
check_vars

# exit if the user is root
check_user

# parse args
set_standard_opts $*

if [ "X$BRIO_PATH" == "X" ]; then
	BRIO_PATH=~/Brio2/TRUNK/
	echo "BRIO_PATH not set.  Using default $BRIO_PATH"
fi

pushd $BRIO_PATH

if [ "$CLEAN" == "1" ]; then
	scons type=embedded -c
	scons type=xembedded -c
fi

# FIXME: use TARGET_MACH and match up with Brio's platform variants!
if [ "$TARGET_CPU" == "LF1000" ]; then
	VARIANT='Lightning_LF1000'
else # XXX
	VARIANT='Lightning_LF2530BLUE'
fi

scons type=embedded platform_variant=$VARIANT
scons type=xembedded platform_variant=$VARIANT

popd
exit 0
