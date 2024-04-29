#!/bin/bash
#
# Make the ROS commands available
# NOTE: these paths should NOT use ~
source /opt/ros/melodic/setup.bash
#source /home/www-share/dfall/dfall-system/dfall_ws/devel/setup.bash
source /home/www-share/dfall/dfall-system/dfall_ws/src/dfall_pkg/launch/Config.sh
#
# Check that the ROS Master exists
# > Note: the -q options converts the 
#   grep output to a true/false
if rosnode list | grep -q /rosout; then
	# Kill all nodes
	rosnode kill --all > /dev/null 2>&1 &
	# The /rosout will respawn, so all kill any
	# "roslaunch" process owned by the
	# "www-data" user
	pkill -u www-data roslaunch
	# For debugging it is usefull to running the
	# following commands on the server machine
	# >> ps -aux
	# >> pgrep -u www-data
	# Inform the user
	echo "Killed all ros nodes"
else
	echo "ROS Master not found"
fi
