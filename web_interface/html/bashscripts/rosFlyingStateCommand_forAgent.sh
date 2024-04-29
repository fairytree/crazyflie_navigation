#!/bin/bash
#
# Check that exactly one command is supplied
if [ "$#" -ne 1 ]; then
	echo "failed"
	exit 1
fi
#
# Check that the command supplied is valid
if [ "$1" != "1" ] && [ "$1" != "2" ] && [ "$1" != "3" ] && [ "$1" != "4" ] && [ "$1" != "5" ] && [ "$1" != "6" ] && [ "$1" != "7" ] && [ "$1" != "8" ] && [ "$1" != "11" ] && [ "$1" != "12" ] && [ "$1" != "13" ]; then
	echo "failed"
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
    if [ "$command" == "1" ]; then
		temp="$(rostopic pub -1 /$ROS_NAMESPACE/$agentnamespace/FlyingAgentClient/Command dfall_pkg/IntWithHeader '{data: 1, shouldCheckForAgentID: False}')"
	fi
	if [ "$command" == "2" ]; then
		temp="$(rostopic pub -1 /$ROS_NAMESPACE/$agentnamespace/FlyingAgentClient/Command dfall_pkg/IntWithHeader '{data: 2, shouldCheckForAgentID: False}')"
	fi
	if [ "$command" == "3" ]; then
		temp="$(rostopic pub -1 /$ROS_NAMESPACE/$agentnamespace/FlyingAgentClient/Command dfall_pkg/IntWithHeader '{data: 3, shouldCheckForAgentID: False}')"
	fi
	if [ "$command" == "4" ]; then
		temp="$(rostopic pub -1 /$ROS_NAMESPACE/$agentnamespace/FlyingAgentClient/Command dfall_pkg/IntWithHeader '{data: 4, shouldCheckForAgentID: False}')"
	fi
	if [ "$command" == "5" ]; then
		temp="$(rostopic pub -1 /$ROS_NAMESPACE/$agentnamespace/FlyingAgentClient/Command dfall_pkg/IntWithHeader '{data: 5, shouldCheckForAgentID: False}')"
	fi
	if [ "$command" == "6" ]; then
		temp="$(rostopic pub -1 /$ROS_NAMESPACE/$agentnamespace/FlyingAgentClient/Command dfall_pkg/IntWithHeader '{data: 6, shouldCheckForAgentID: False}')"
	fi
	if [ "$command" == "7" ]; then
		temp="$(rostopic pub -1 /$ROS_NAMESPACE/$agentnamespace/FlyingAgentClient/Command dfall_pkg/IntWithHeader '{data: 7, shouldCheckForAgentID: False}')"
	fi
	if [ "$command" == "8" ]; then
		temp="$(rostopic pub -1 /$ROS_NAMESPACE/$agentnamespace/FlyingAgentClient/Command dfall_pkg/IntWithHeader '{data: 8, shouldCheckForAgentID: False}')"
	fi
	if [ "$command" == "11" ]; then
		temp="$(rostopic pub -1 /$ROS_NAMESPACE/$agentnamespace/FlyingAgentClient/Command dfall_pkg/IntWithHeader '{data: 11, shouldCheckForAgentID: False}')"
	fi
	if [ "$command" == "12" ]; then
		temp="$(rostopic pub -1 /$ROS_NAMESPACE/$agentnamespace/FlyingAgentClient/Command dfall_pkg/IntWithHeader '{data: 12, shouldCheckForAgentID: False}')"
	fi
	if [ "$command" == "13" ]; then
		temp="$(rostopic pub -1 /$ROS_NAMESPACE/$agentnamespace/FlyingAgentClient/Command dfall_pkg/IntWithHeader '{data: 13, shouldCheckForAgentID: False}')"
	fi
    #
    # Return that the message was sent
    echo "sent"
else
    echo "ROS Master not found"
fi
