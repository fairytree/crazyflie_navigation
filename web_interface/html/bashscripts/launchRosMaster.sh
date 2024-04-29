#!/bin/bash
#
# Check that exactly one command is supplied
if [ "$#" -ne 1 ]; then
	echo "exactly one argument must be supplied"
	exit 1
fi
#
# Check that the first command supplied is valid
if [ "$1" != "true" ] && [ "$1" != "false" ]; then
	echo "emulate mocap = $1, is not a valid boolean."
	exit 1
fi
#
# Put the commands into variables for make things more readable
emultemocap=$1
#
# Make the ROS commands available
# NOTE: these paths should NOT use ~
source /opt/ros/melodic/setup.bash
source /home/www-share/dfall/dfall-system/dfall_ws/devel/setup.bash
source /home/www-share/dfall/dfall-system/dfall_ws/src/dfall_pkg/launch/Config.sh
#
# Set the ROS log location
export ROS_LOG_DIR=/var/www/html/.ros/log
#
# Mount the dfall workspace folder
#sudo mount --bind /home/pbeuchat/gitrep/dfall/dfall-system/dfall_ws /var/www/html/dfall_ws
#
# Check that the ROS Master exists
# > Note: the -q options converts the
#   grep output to a true/false
if rosnode list | grep -q /rosout; then
	echo "ROS Master is already running"
else
	nohup roslaunch dfall_pkg pi_master.launch emulateMocap:=$emultemocap > /dev/null 2>&1 &
	echo "ROS Master successfully launched"
fi