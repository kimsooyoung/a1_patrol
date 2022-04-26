#!/bin/bash

export SUDO_ASKPASS=/home/unitree/catkin_ws/src/slamrplidarV2.0/slam_planner/scripts/PASSWD

IDsdk=`ps -ef | grep "slamwalk_node" | grep -v "$0" | grep -v "grep" | awk '{print $2}'`
 if [ "$IDsdk" != "" ]; then
     sudo -A kill $IDsdk
 fi

ID=`ps -ef | grep 'roslaunch slam_planner' | grep -v '$0' | grep -v 'grep' | awk '{print $2}'`
if [ "$ID" != "" ]; then
	sudo -A kill $ID
	echo "Killed"
else 
	echo "Nothing"
fi
