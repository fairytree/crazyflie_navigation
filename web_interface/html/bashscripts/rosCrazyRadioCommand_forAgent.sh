#!/bin/bash
#
# Check that exactly one command is supplied
if [ "$#" -ne 1 ]; then
	echo "exactly one argument must be supplied"
	exit 1
fi
#
# Check that the command supplied is valid
if [ "$1" != "0" ] && [ "$1" != "1" ]; then
	echo "command = $1, is not a valid option"
	exit 1
fi
#
# Put the command into a variable for make things more readable
command=$1
#
# Make the ROS commands available
# NOTE: these paths should NOT use ~
source /opt/ros/melodic/setup.bash
source /home/www-share/dfall/dfall-system/dfall_ws/devel/setup.bash
source /home/www-share/dfall/dfall-system/dfall_ws/src/dfall_pkg/launch/Config.sh
#
# Check that the ROS Master exists
# > Note: the -q options converts the
#   grep output to a true/false
if rosnode list | grep -q /rosout; then
	# Convert the agent ID to a zero padded string
	agentnamespace=$(printf "agent%03d" $DFALL_DEFAULT_AGENT_ID)
	# Send the message
	if [ "$command" == "0" ]; then
		# Publish the request
		temp="$(rostopic pub -1 /$ROS_NAMESPACE/$agentnamespace/FlyingAgentClient/CrazyRadioCommand dfall_pkg/IntWithHeader "{data: 0, shouldCheckForAgentID: False}")"
		# Return that the message was sent
		echo "sent"
	#
	elif [ "$command" == "1" ]; then
		# Publish the request
		temp="$(rostopic pub -1 /$ROS_NAMESPACE/$agentnamespace/FlyingAgentClient/CrazyRadioCommand dfall_pkg/IntWithHeader "{data: 1, shouldCheckForAgentID: False}")"
		# Return that the message was sent
		echo "sent"
	else
		# Return that the command is not recognised
		echo "command = $command is not a valid option"
	fi

else
	echo "ROS Master not found"
fi
