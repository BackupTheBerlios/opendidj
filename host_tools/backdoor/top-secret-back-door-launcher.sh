#!/bin/sh

# This is the top secret back door launcher.  It is used as a generic back door
# launcher for random scripts that need to be run by develolpers who do not have
# access to a terminal prompt.  To use it, back up Didj:Base/bin/AppManager and
# simply paste this script it its place.  Be sure to rename it AppManager so the
# system invokes it.  Finally, power cycle the board.  The script simply runs
# all programs named /Didj/*.backdoor on the target in sorted order and pipes
# their output to the file /Didj/backdoor-results.  If the
# /Didj/backdoor-results file exists when the launcher runs, it quits assuming
# that its tasks are complete.  You can run the script again by deleting the
# file.  You can, of course, brick your board depending on the contents of the
# scripts.

# Simply quit if the usb is connected.
USB_CONNECTED=`cat /sys/devices/platform/lf1000-usbgadget/vbus`

if [ "$USB_CONNECTED" != "0" ]; then
	echo "Can't execute with USB connected."
	exit 0
fi

# Quit if the /Didj/backdoor-results script exists
LOGFILE=/Didj/backdoor-results
if [ -e $LOGFILE ]; then
	echo "Looks like we already ran!  Go to system restore mode"
	exit 125
else
	touch $LOGFILE
fi

echo "Created log file"

# Okay.  The usb is not connected.  Time to run all of the scripts
for p in `ls /Didj/*.backdoor 2>/dev/null | sort`; do
	if [ -f $p ]; then
		echo "Running $p" | tee -a $LOGFILE
		$p 2>&1 | tee -a $LOGFILE
	fi
done
