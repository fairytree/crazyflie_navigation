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
	echo "emulate crazyradio = $1, is not a valid boolean."
	exit 1
fi
#
# Put the commands into variables for make things more readable
emulateradio=$1
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
	# Check that if the agent is already
	# launched
	if rosnode list | grep -q "$(printf "/dfall/agent%03d" $DFALL_DEFAULT_AGENT_ID)"; then
		echo "Agent $DFALL_DEFAULT_AGENT_ID is already running"
	else
		nohup roslaunch dfall_pkg pi_agent.launch emulateRadio:=$emulateradio > /dev/null 2>&1 &
		echo "Agent $DFALL_DEFAULT_AGENT_ID successfully launched"
	fi
else
	echo "ROS Master not found"
fi
