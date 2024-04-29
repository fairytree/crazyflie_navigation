#!/bin/bash
#
# Check that exactly five commands are supplied
if [ "$#" -ne 5 ]; then
	echo "exaclty five arguments must be supplied"
	exit 1
fi
#
# Check that the first command supplied is valid
if [ "$1" != "default" ] && [ "$1" != "student" ]; then
	echo "command = $1, is not a valid option."
	exit 1
fi
#
# Put the commands into variables for make things more readable
command=$1
xnew=$2
ynew=$3
znew=$4
yawnew=$5
#
# Check that the new setpoint values are purely numerical
# > For x:
if ! [[ $xnew =~ ^[+-]?[0-9]+\.?[0-9]*$ ]];then
	echo "x = $xnew, is not a float."
	exit 1
fi
# > For y:
if ! [[ $ynew =~ ^[+-]?[0-9]+\.?[0-9]*$ ]];then
	echo "y = $ynew, is not a float."
	exit 1
fi
# > For z:
if ! [[ $znew =~ ^[+-]?[0-9]+\.?[0-9]*$ ]];then
	echo "z = $znew, is not a float."
	exit 1
fi
# > For yaw:
if ! [[ $yawnew =~ ^[+-]?[0-9]+\.?[0-9]*$ ]];then
	echo "yaw = $yawnew, is not a float."
	exit 1
fi
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
	if [ "$command" == "default" ]; then
		# Publish the request
		temp="$(rostopic pub -1 /$ROS_NAMESPACE/$agentnamespace/DefaultControllerService/RequestSetpointChange dfall_pkg/SetpointWithHeader "{x: $xnew, y: $ynew, z: $znew, yaw: $yawnew, shouldCheckForAgentID: False}")"
		# Return that the message was sent
		echo "sent"
	#
	elif [ "$command" == "student" ]; then
		# Publish the request
		temp="$(rostopic pub -1 /$ROS_NAMESPACE/$agentnamespace/StudentControllerService/RequestSetpointChange dfall_pkg/SetpointWithHeader "{x: $xnew, y: $ynew, z: $znew, yaw: $yawnew, shouldCheckForAgentID: False}")"
		# Return that the message was sent
		echo "sent"
	#
	else
		# Return that the command is not recognised
		echo "controller = $command is not a valid option"
	fi

else
	echo "ROS Master not found"
fi

# DEBUGGING: For testing directly in terminal
# rostopic pub -1 /dfall/agent001/DefaultControllerService/RequestSetpointChange dfall_pkg/SetpointWithHeader {x: 0.0, y: 0.0, z: 0.1, yaw: 0, shouldCheckForAgentID: False}