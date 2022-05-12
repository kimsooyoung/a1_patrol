# slamrplidar

  A SLAM and Path PLanning system based a single rplidar(with IMU).
 
  `Some Information about the hardware connect, please visit **********`

## 1 Running (Local Login)

1. First, start the laikago, waiting to communication:
    `$ cd ~/aliengo_sdk/build`
    `$ sudo ./sdk_lcm_server_high`

2. Open a new terminal, start the rplidar and mapping:
    `$ roslaunch slam_planner slam_rplidar_start.launch`

3. In another terminal, launch the Path planning algorithm:
    `$ roslaunch slam_planner slam_planner_online.launch`

4. In another terminal, load the base_controller_node node to control laikago(*Auto: planning...*):
    `$ rosrun slam_planner base_controller_node`

## 2 Running (Remote Login)

`It's a remote login test, "ssh"`

1. First, login in the laikago's computer (ssh your_laikago's_ip_address), open two terminals:

    < Notice: Keep your computer with laikago's computer in a LAN. >

    `do 1.1, 1.2`

2. Then, open a new terminal: (ssh -X your_laikago's_ip_address)

    `do 1.3`

3. In another terminal, (ssh your_laikago's_ip_address):

    `do 1.4`

After complete above steps, the 2D-Map will be shown in the rviz.

And, you can use the `2D Nav Goal` button(or publish a message to `/move_base_simple/goal` topic ) to set the goal position and orientation.

The laikago will search the optimal path to reach the target point, with avoiding the dynamic obstacles.




