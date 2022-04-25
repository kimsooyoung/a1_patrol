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


```
remote: Invalid username or password.


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

```