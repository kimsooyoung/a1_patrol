# a1_patrol

```
roslaunch unitree_legged_real real.launch
roslaunch start joy_control.launch
```

```
roslaunch unitree_legged_real real.launch
roslaunch start twist_control.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

```
roslaunch slam_planner slam_rplidar_start.launch
rostopic pub /slamware_ros_sdk_server_node/clear_map slamware_ros_sdk/ClearMapRequest "kind:
 kind: 0"
```

```
roslaunch unitree_legged_real real.launch
roslaunch start twist_control_nav.launch
roslaunch slam_planner slam_planner_online.launch
roslaunch start view_nav.launch
```

```
roslaunch unitree_legged_real real.launch
roslaunch start twist_control_realsense.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

roslaunch realsense2_camera rs_camera.launch
rosrun image_view image_view image:=<your topic name>
rosbag record -a -o a1_realsense_hanyang
```



[] 2D lidar compile && check
[] Realsense check && example coding
[] ROS 2 & Navigation
[] 3D Lidar Documentation
[] Docker
[] 