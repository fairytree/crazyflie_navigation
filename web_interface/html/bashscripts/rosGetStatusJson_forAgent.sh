#!/bin/bash
#
#
# Check whether a command is supplied
# > and put it into a more redable variable
if [ "$#" -eq 0 ]; then
	eventid="0"
elif [ "$#" -eq 1 ]; then
	eventid=$1
else
	echo "not allowed to supply two or more arguments"
	exit 1
fi
#
# Check that the command supplied is valid
if ! [[ $eventid =~ ^[0-9]*$ ]]; then
	echo "event id must be an integer, event id = $1"
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
	# Check if the agent exists
	if rosnode list | grep -q "$(printf "/dfall/agent%03d" $DFALL_DEFAULT_AGENT_ID)"; then
		# Set a flag that agent was found
		agentfound="true"
		# Convert the agent ID to a zero padded string
		agentnamespace=$(printf "agent%03d" $DFALL_DEFAULT_AGENT_ID)    
		# Perform the service call for the status json
		statusjsonfull="$(rosservice call /$ROS_NAMESPACE/$agentnamespace/AgentStatusForWebInterface/StatusAsJson 0)"
		# Cut out the "data: " from the start, and the single " at the end
		statusjsoncut1="${statusjsonfull:7:${#statusjsonfull}-8}"
		# Remove all new line characters followed by 2 spaces
		statusjsoncut2=$(echo $statusjsoncut1|tr -d '\n  ')
		# Remove all remaining new line characters
		statusjsoncut3=$(echo $statusjsoncut2|tr -d '\n')
		#statusjsoncut3=${statusjsoncut2//$'\n'/}
		# Remove all carriage return characters
		statusjsoncut4=$(echo $statusjsoncut3|tr -d '\r')
		# Remove all tab characters
		statusjsoncut5=$(echo $statusjsoncut4|tr -d '\t')
		# Remove all remaining \ escape characters
		statusjson=${statusjsoncut5//\\}
		#
		# Check that the status json is valid based on length
		if [ ${#statusjsonfull} -le 1 ]; then
			# Set a flag that agent was NOT found
			agentfound="false"
			# Set all other values to "not available (na)"
			statusjson="\"na\""
		fi
		#
	else
		# Set a flag that agent was NOT found
		agentfound="false"
		# Set all other values to "not available (na)"
		statusjson="\"na\""
	fi
else
	# Set a flag that agent was NOT found
	agentfound="false"
	# Set all other values to "not available (na)"
	statusjson="\"na\""
fi

# Return that the values collected
echo "{"\
	"\"id\": \"$eventid\","\
	"\"agentfound\": \"$agentfound\","\
	"\"statusjson\": $statusjson"\
	"}"

# DEBUGGING: For testing directly in terminal
# rosservice call /dfall/agent001/AgentStatusForWebInterface/StatusAsJson 0