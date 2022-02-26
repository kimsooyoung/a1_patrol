#! /bin/bash
echo "Usage: ./getMap.sh yourMapName"

python3 /home/nuc/catkin_ws/src/ndt_localization/prm_localization/script/convert_pcd.py --map_name=$1
echo "Successfully convert pcd map --> $1.pcd"
python3 /home/nuc/catkin_ws/src/ndt_localization/prm_localization/script/generate_localization_launch.py --map_name=$1
echo "Successfully generate new launch file for localization"