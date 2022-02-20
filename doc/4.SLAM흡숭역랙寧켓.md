# SLAM软件开发指南

## 目录

- [SLAM软件开发指南](#slam软件开发指南)
  - [目录](#目录)
  - [1. 平台](#1-平台)
  - [2. 软件依赖及安装](#2-软件依赖及安装)
    - [更换 apt 源](#更换-apt-源)
    - [增加 swap 空间](#增加-swap-空间)
    - [2.1. ROS](#21-ros)
    - [2.2. LIO-SAM 运行依赖](#22-lio-sam-运行依赖)
    - [2.3. Unitree 程序运行依赖](#23-unitree-程序运行依赖)
    - [2.4. 其他环境配置](#24-其他环境配置)
  - [3. 运行使用](#3-运行使用)
    - [3.1. 编译及环境配置](#31-编译及环境配置)
    - [3.2. 启动任务](#32-启动任务)
  - [4. 远程连接及可视化](#4-远程连接及可视化)
    - [4.1. 远程控制](#41-远程控制)
    - [4.2. 远程可视化](#42-远程可视化)
  - [5. 开发说明](#5-开发说明)
  - [Linux 常用命令](#linux-常用命令)

## 1. 平台

Aliengo 机器狗（miniPC with Ubuntu 16.04），Vledyne VLP-16 激光雷达

## 2. 软件依赖及安装

### 更换 apt 源

打开存放源地址的文件 `/etc/apt/source.list` 修改源地址

打开源地址文件

```
sudo nano /etc/apt/source.list
```

注释里面的所有内容，将下列内容复制到文件里

```
deb http://mirrors.aliyun.com/ubuntu/ xenial main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ xenial-security main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ xenial-updates main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ xenial-proposed main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ xenial-backports main restricted universe multiverse

deb-src http://mirrors.aliyun.com/ubuntu/ xenial main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ xenial-security main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ xenial-updates main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ xenial-proposed main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ xenial-backports main restricted universe multiverse
```

更新软件源

```
sudo apt-get update
```

### 增加 swap 空间

`miniPC` 默认内存只有 4G，对于编译有点小，因此需要增加 swap 空间，以便快速通过编译。每次重启后 `swap` 空间会被删除，可使用 `free -h` 检查

```
#count的大小就是增加的swap空间的大小，64M是块大小，所以空间大小是bs*count=16384MB=16GB
sudo dd if=/dev/zero of=/swapfile bs=64M count=256

# 修改文件权限，只能被root访问
sudo chmod 600 /swapfile

#把刚才空间格式化成swap格式
sudo mkswap /swapfile

#使用刚才创建的swap空间
sudo swapon /swapfile

# 查看 swap 分区情况
free -h
```

### 2.1. ROS

安装 `ROS`，摘录自： http://wiki.ros.org/kinetic/Installation/Ubuntu

```
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
```

```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

```
sudo apt update
```

```
sudo apt install ros-kinetic-desktop-full
```

```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

设置环境变量

```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

安装所需要的 ROS 包

```
sudo apt-get install -y ros-kinetic-slam-gmapping
sudo apt-get install -y ros-kinetic-roslint
sudo apt-get install -y ros-kinetic-velodyne
sudo apt-get install -y ros-kinetic-teb-local-planner
sudo apt-get install -y libpcap-dev
sudo apt-get install -y ros-kinetic-navigation
sudo apt-get install -y ros-kinetic-robot-localization
sudo apt-get install -y ros-kinetic-robot-state-publisher
```

### 2.2. LIO-SAM 运行依赖

摘录自： https://github.com/TixiaoShan/LIO-SAM

[GTSAM](https://github.com/borglab/gtsam/releases): Georgia Tech Smoothing and Mapping library

```
wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.2.zip
cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
cd ~/Downloads/gtsam-4.0.2/
mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
sudo make install -j8
```

如果使用 `sudo make install -j8` 编译失败，可改用 `sudo make install -j1` 试试

### 2.3. Unitree 程序运行依赖

摘录自：

https://github.com/unitreerobotics/unitree_ros

https://github.com/unitreerobotics/unitree_legged_sdk

https://lcm-proj.github.io/build_instructions.html

依赖： 

 - [`Boost`](https://www.boost.org/) ( version 1.5.4 or higher )
 - [`CMake`](https://cmake.org/) ( version 2.8.3 or higher )，
 - [`LCM`](https://lcm-proj.github.io/index.html) ( version 1.4.0 or higher )
 - [`unitree_legged_sdk`](https://github.com/unitreerobotics/unitree_legged_sdk)
 
`Boost` 与 `Cmake` 一般都有，不再赘述。

安装 `LCM`： 从[官方指定网址](https://github.com/lcm-proj/lcm/releases)下载 `lcm-1.4.0.zip` ，解压文件，然后执行

```
cd lcm-1.4.0
mkdir build
cd build
cmake ../
make
sudo make install
```

安装 `unitree_legged_sdk`

下载与编译

```
cd ~
git clone https://github.com/unitreerobotics/unitree_legged_sdk.git
cd unitree_legged_sdk
mkdir build
cd build
cmake ../
make
```

配置环境变量
```
echo "export UNITREE_LEGGED_SDK_PATH=~/unitree_legged_sdk" >> ~/.bashrc
echo "export ALIENGO_SDK_PATH=~/aliengo_sdk" >> ~/.bashrc

```

### 2.4. 其他环境配置

```
#amd64, arm32, arm64
echo "export UNITREE_PLATFORM=amd64" >> ~/.bashrc
```

打开 `~/.bashrc` 文件

```
nano ~/.bashrc
```

将所有的 `catkin_ws` 替换为 `patroldog_ws`

## 3. 运行使用

登陆 `miniPC`，将 `patroldog_ws` 文件夹放到主目录（即 `~` 目录）下

### 3.1. 编译及环境配置

进入 `patroldog_ws` 文件夹下运行命令

```
catkin_make
```

运行程序时，需要将 `patroldog_ws` 添加到 `ROS` 的工作路径（使用以下命令，只需执行一次）

```
echo "source ~/patroldog_ws/build/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

运行 `unitree_legged_real` 需要 `root` 权限，因此启动任务时需要首先切换到超级用户

```
sudo su
```

### 3.2. 启动任务

启动建图任务：

```
roslaunch start build_map.launch map_name:=my_map_name
```

建图时可使用控制器上的`X`键设置巡逻点，巡逻点保存在文件夹 `/home/maps/gmapping` 下的 `txt` 文件里，每一行为一个巡逻点，每一行的三个值分别为：`x`，`y`，`yaw`（角度），`time`（停留时间）

启动巡逻任务:

```
roslaunch start patrol.launch map_name:=my_map_name
```

巡逻时需要在建图的起始位置启动巡逻任务（否则无法准确定位），机器狗会按照建图时设置的巡逻点依次巡逻。

## 4. 远程连接及可视化

### 4.1. 远程控制

查看本机 `ip` 地址： 

```
ifconfig
```

使 miniPC 和控制电脑处在同一局域网下，然后使用 `ssh` 登录 miniPC ，使用命令行控制机器狗。假设miniPC 的IP地址为：`192.168.1.64`，则可使用下面指令登陆 miniPC，`unitree`为 miniPC 的用户名

```
ssh unitree@192.168.1.64
```

### 4.2. 远程可视化

建图和巡逻时可以远程可视化机器狗当前对空间的感知情况，并能在建图时远程控制机器狗的运动目标。

同样地，使 miniPC 和控制电脑处在同一局域网下，分别配置 miniPC 与控制电脑的 `ROS_MASTER_URI` 与 `ROS_IP`，即可在控制电脑上使用 `Rviz` 远程查看当前的感知情况（如地图，激光雷达，路径等）

例如

```
# 假如
# 控制电脑 IP： 192.168.1.36
# miniPC IP： 192.168.1.64

# 在 miniPC 的命令行中输入
echo "export ROS_MASTER_URI=http://192.168.12.131:11311" >> ~/.bashrc
echo "export ROS_IP=192.168.12.131" >> ~/.bashrc
source ~/.bashrc

# 在控制电脑的命令行中输入
echo "export ROS_MASTER_URI=http://192.168.12.131:11311" >> ~/.bashrc
echo "export ROS_IP=192.168.12.178" >> ~/.bashrc
source ~/.bashrc
```

注意：使用某一用户进行的配置仅对该用户有效，即如果使用 root 用户启动任务，则需要使用 root 执行上述配置

完成上述配置之后，可在本地执行下列指令，以验证是否配置成功（需要启动远程端的`roscore`）

```
rostopic list
```

若输出了远程端应有的 `topic`，则配置成功

若配置成功，则可在本地执行下列指令以启动可视化软件 `Rviz`

```
rosrun rviz rviz
```

启动 `Rviz` 时，可加载提前配置好的文件（在 `patroldog_ws` 文件夹下的特定位置）来展示所需要的信息，即

```
# 假设 patroldog_ws 被放在 /home/unitree/ 路径下

# 建图时使用
rosrun rviz rviz -d /home/unitree/patroldog_ws/src/lio_sam/launch/include/config/rviz.rviz

# 巡逻时使用
rosrun rviz rviz -d /home/unitree/patroldog_ws/src/ndt_localization/prm_localization/rviz/result.rviz
```

## 5. 开发说明

本套程序建图时使用 `LIO-SAM` 算法作为激光雷达 SLAM 算法，其紧密耦合了激光雷达的数据和狗自身反馈的 `IMU` 数据，实现了同步定位与建图功能。如需进一步了解 `LIO-SAM` 算法并在此基础上开发，可参考[原论文](https://arxiv.org/abs/2007.00258)以及[Github仓库](https://github.com/TixiaoShan/LIO-SAM)中的介绍。

路径规划以及避障使用了 ROS 的 `navigation` 包，并以 `teb_local_planner` 作为局部路径规划器。

同时使用了 `gmapping` 构建 2D 全局地图以进行全局路径规划（ LIO-SAM 也会生成全局地图，并被用于巡逻时的定位，但其不适用于有高度差环境的全局路径规划）。

巡逻时会加载建图时构建好的地图，使用 `NDT` 算法进行 3D 点云匹配定位，此算法需要提供大概的初始位姿信息（默认所有值为 0 ，因此启动巡逻时的机器狗初始位姿需要与启动建图时的初始位姿相同），巡逻时不会运行 `LIO-SAM` 算法。

各软件包解读

```
 - gmapping  # 构建用于全局路径规划的2D全局地图，与源程序不同，源码有所修改，gmapping 的 odom 使用 lio-sam 的 odom
 - lio_sam  # SLAM 算法，参数在 config/params.yaml 下修改，源码有所修改
 - navigation  # 调用 move_base 的 launch 文件以及参数配置，机器人的运动表现很大程度取决于这里的配置
 - ndt_localization  # 巡逻时的定位算法
 - start  # 启动任务的 launch 文件，以及一些起“胶水”作用的小程序，比如巡逻点的发布
 - unitree_legged_msgs  # 读取机器狗返回数据所需的消息类型
 - unitree_legged_real  # 与机器狗底层沟通所需要的程序
 - velodyne  # 启动激光雷达的驱动程序
```

建图时的程序框图

![alt build_map](./build_map_draw.png)

巡逻时的程序框图

![alt start_patrol](./start_patrol_draw.png)

建图时的 `ROS` 节点图

![alt build_map](./build_map.png)

巡逻时的 `ROS` 节点图

![alt start_patrol](./start_patrol.png)



