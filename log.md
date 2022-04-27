# Installation

```
sudo apt-get install python-pip
sudo pip install -U rosdep

cd a1_patrol
sudo rosdep init
rosdep update

sudo apt install ros-melodic-joy

```

Unitree_legged_sdk Setup

```
mkdir -p /home/unitree/Unitree/sdk
cd /home/unitree/Unitree/sdk
clone legged_sdk & build
```

SLAMWare SDK

```
sudo apt-get install build-essential gcc make cmake cmake-gui cmake-curses-gui
sudo apt-get install libssl-dev 
sudo apt-get install doxygen graphviz

slamware_ros_sdk & slamware_sdk => catkin_ws 안에 두고 빌드
slam_planner & slam_planner_sdk => 포함

cop slamware_ros_sdk && sds
cop slam_planner && sds
```

realsense

```
sudo apt-get install ros-melodic-realsense2-camera -y
sudo apt install ros-melodic-ddynamic-reconfigure -y

# librealsense
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils

Reconnect the Intel RealSense depth camera and run: realsense-viewer to verify the installation.

confirm connection with realsense-viewer 
[picture]

# realsense-ros

cd realsense_ws
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install

cop realsense2_description -DCMAKE_BUILD_TYPE=Release
cop realsense2_camera -DCMAKE_BUILD_TYPE=Release
catkin_make install

roslaunch realsense2_camera rs_camera.launch
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
[picture]
```

bashrc utils

```
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=127.0.0.1

alias eb='gedit ~/.bashrc'
alias sb='source ~/.bashrc'

alias cba='colcon build --symlink-install'
alias cbp='colcon build --symlink-install --packages-select'
alias killg='killall -9 gzserver && killall -9 gzclient && killall -9 rosmaster'

alias cma='catkin_make -DCATKIN_WHITELIST_PACKAGES=""'
alias cop='catkin_make --only-pkg-with-deps'
alias copr='catkin_make -DCMAKE_BUILD_TYPE=Release --only-pkg-with-deps'
alias sds='source devel/setup.bash'
alias axclient='rosrun actionlib axclient.py'

alias rosmelo='source /opt/ros/melodic/setup.bash'
alias roseloq='source /opt/ros/eloquent/setup.bash && source ./install/setup.bash && export PYTHONPATH=/opt/ros/eloquent/lib/python3.6/site-packages'
alias rosdinstall='rosdep install -y -r -q --from-paths src --ignore-src --rosdistro'
```


Build

```
cop unitree_legged_msgs && sds
cop unitree_legged_real && sds
cop unitree_joy_cmd && sds
cop unitree_twist_cmd && sds
```

시작하기

시작하기 전에 랜선 뽑고 부팅

부팅 => L2 + A => L2 + Start => L1 + Start 제어


부팅 => (L1 + start)

L2 + A 하고 나서 L2 + Start로 ROS 제어 진입할 수 있는데 이때 조이스틱 왼쪽이 중립을 잘 유지해야 바뀐다.

```
ifsetup 192.168.123.12

roslaunch unitree_legged_real real.launch
rosrun unitree_legged_real ros_control_helper

rostopic echo /imu_raw => check imu sub
```

```
roslaunch unitree_legged_real real.launch
roslaunch start joy_control.launch
```

```
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://192.168.0.4:43885/
ros_comm version 1.14.10


SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.10

NODES

auto-starting new master
process[master]: started with pid [12597]
ROS_MASTER_URI=http://192.168.0.4:11311/

setting /run_id to 70fe5b32-c581-11ec-a91b-48b02d078d9b
process[rosout-1]: started with pid [12608]
started core service [/rosout]

```


```lcm_server_high.cpp
void Custom::LCMRecv()
{
    if(mylcm.highCmdLCMHandler.isrunning){
        pthread_mutex_lock(&mylcm.highCmdLCMHandler.countMut);
        mylcm.highCmdLCMHandler.counter++;
        if(mylcm.highCmdLCMHandler.counter > 1000){
            printf("Error! LCM Time out.\n");
            exit(-1);              // can be commented out
        }
        pthread_mutex_unlock(&mylcm.highCmdLCMHandler.countMut);
    }
    mylcm.Recv();
}
```