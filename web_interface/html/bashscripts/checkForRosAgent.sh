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
    # Check if the agent exists
    if rosnode list | grep -q "$(printf "/dfall/agent%03d" $DFALL_DEFAULT_AGENT_ID)"; then
        echo "Agent $DFALL_DEFAULT_AGENT_ID exists"
    else
        echo "Agent $DFALL_DEFAULT_AGENT_ID not found"
    fi
else
    echo "ROS Master not found"
fi
