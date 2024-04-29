#!/bin/bash
#
# Make the ROS commands available
# NOTE: these paths should NOT use ~
source /opt/ros/melodic/setup.bash
#source /home/www-share/dfall/dfall-system/dfall_ws/devel/setup.bash
source /home/www-share/dfall/dfall-system/dfall_ws/src/dfall_pkg/launch/Config.sh
#
# Change directory to the dfall-system repository
cd /home/www-share/dfall/dfall-system/dfall_ws
#
# Call catkin_make
catkin_make 2>&1