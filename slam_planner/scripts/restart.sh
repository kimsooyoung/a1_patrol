#!/bin/sh
##############
# Author: ian
##############
#sleep 10

export SUDO_ASKPASS=/home/unitree/catkin_ws/src/slamrplidarV2.0/slam_planner/scripts/PASSWD

ID=`ps -ef | grep 'roslaunch slam_planner' | grep -v '$0' | grep -v 'grep' | awk '{print $2}'`
if [ "$ID" != "" ]; then
	sudo -A kill $ID
	echo "Killed"
else 
	echo "Nothing"
fi

IDsdk=`ps -ef | grep "slamwalk_node" | grep -v "$0" | grep -v "grep" | awk '{print $2}'`
 if [ "$IDsdk" != "" ]; then
     sudo -A kill $IDsdk
 fi


gnome-terminal -- bash -c "source /opt/ros/melodic/setup.bash; source /home/unitree/catkin_ws/devel/setup.bash; roslaunch slam_planner slam_planner_online.launch; exec bash"
plannerID=$(ps -ef | grep slam_planner | grep -v "grep" | wc -l)

sleep 5;
gnome-terminal -- bash -c "cd /home/unitree/catkin_ws/devel/lib/slam_planner; sudo -AE ./slamwalk_node; exec bash"


if [ $plannerID -ge 1 ]; then
	echo "SUCCESS"
else
	echo "sth is error, please restart the application !"
fi
