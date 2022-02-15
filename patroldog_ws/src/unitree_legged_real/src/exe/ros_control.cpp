/************************************************************************
Added by Syx, control dog with ros
************************************************************************/

#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <pthread.h>
#include <string>
#include <fstream>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/IMU.h>
// #include "aliengo_sdk/aliengo_sdk.hpp"
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include "convert.h"

// 设置一些参数
float linear_x = 0.0;
float linear_y = 0.0;
float angular_z = 0.0;
uint8_t mode = 1;
bool can_move = false;
bool is_build_map = false;
float yaw = 0;

// 创建狗 imu 的数据
// dog's imu message to publish
sensor_msgs::Imu dogImu;
std_msgs::Bool dog_can_move;

// 从 ros 读取运动指令的回调函数
void cmd_vel_cb(const geometry_msgs::Twist& msg){
    linear_x = msg.linear.x;
    linear_y = msg.linear.y;
    angular_z = msg.angular.z;
}

// 从 ros 读取运动指令的回调函数
void mode_cb(const std_msgs::UInt8& msg){
    mode = msg.data;
}

// 一个 LCM 的模板
template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

// main 函数的模板
template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();
    
    // 获取 ros 的参数
    ros::param::get("dog_control_node/is_build_map", is_build_map);
    // 设置参数 巡逻点文件 的默认值
    std::string patrol_points_file_path = "";
    // 判断是否是建图的任务，否则就不需要保存巡逻点
    if (is_build_map) {
        ros::param::get("dog_control_node/patrol_points_file", patrol_points_file_path);
    }
    // 读取巡逻点文件
    std::ofstream patrol_points_file(patrol_points_file_path, std::ios::trunc);
    int last_start_cmd = 0;
    int last_set_cmd = 0;
    float last_point[3] = {0};
    bool is_same_point = false;

    // 创建一些坐标变换的变量
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;

    // ros 句柄与参数设置
    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    // 设置一些 topic 的发布器和订阅器
    ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 1, cmd_vel_cb);
    ros::Subscriber mode_sub = n.subscribe("/dog_mode", 1, mode_cb);
    ros::Publisher dog_imu_pub = n.advertise<sensor_msgs::Imu>("/imu_raw", 1000);
    ros::Publisher dog_can_move_pub = n.advertise<std_msgs::Bool>("/dog_can_move", 1000);

    // 与狗通信的一些设置
    // SetLevel(HIGHLEVEL);
    TCmd SendHighLCM = {0};
    TState RecvHighLCM = {0};
    unitree_legged_msgs::HighCmd SendHighROS;
    unitree_legged_msgs::HighState RecvHighROS;

    // roslcm 的初始化
    roslcm.SubscribeState();

    // 开启一个新的线程
    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    // 当 ros 正常时，进入循环
    while (ros::ok()){
        // 通过 LCM 获取狗发来的信息
        roslcm.Get(RecvHighLCM);
        // 将获取的数据解码到 ros 的格式
        RecvHighROS = ToRos(RecvHighLCM);
        // printf("%f\n",  RecvHighROS.forwardSpeed);
        // for(int i=0; i < 40; i++) {
        //     std::cout << i << "  RECV: " << int(RecvHighROS.wirelessRemote[i]) << std::endl;
        // }
        // 通过 遥控器的 start 键控制狗是否能动
        if (RecvHighROS.wirelessRemote[2]-last_start_cmd==4) {
            can_move = !can_move;
            if (can_move) {
                mode = 2;
            }
            std::cout << "!!!!! can move: " << bool(can_move) << std::endl;
        }

        // 构建并发布狗是否能动的信息到 ros topic 中
        dog_can_move.data = can_move;
        dog_can_move_pub.publish(dog_can_move);
        // std::cout << int(RecvHighROS.wirelessRemote[2])  << " mode: " << int(mode) << std::endl;
        last_start_cmd = RecvHighROS.wirelessRemote[2];
        
        // 构建机器狗 imu 的数据
        dogImu.header.frame_id = "imu_link";
        dogImu.header.stamp = ros::Time::now();
        // dog's orientation 狗的方向信息
        dogImu.orientation.w = RecvHighROS.imu.quaternion[0];
        dogImu.orientation.x = -RecvHighROS.imu.quaternion[1];
        dogImu.orientation.y = -RecvHighROS.imu.quaternion[2];
        dogImu.orientation.z = -RecvHighROS.imu.quaternion[3];
        // dogImu.orientation_covariance[0] = -1;
        // dog's angular_velocity 狗的角速度信息
        dogImu.angular_velocity.x = RecvHighROS.imu.gyroscope[0];
        dogImu.angular_velocity.y = RecvHighROS.imu.gyroscope[1];
        dogImu.angular_velocity.z = RecvHighROS.imu.gyroscope[2];
        // dog's linear_acceleration 狗的线性加速度信息
        dogImu.linear_acceleration.x = RecvHighROS.imu.accelerometer[0];
        dogImu.linear_acceleration.y = RecvHighROS.imu.accelerometer[1];
        dogImu.linear_acceleration.z = RecvHighROS.imu.accelerometer[2];
        // publish dog's imu message
        // 发布狗的 imu 信息到 ros 中，供别的节点使用
        dog_imu_pub.publish(dogImu);
        // 如果是建图过程，则执行保存巡逻点的程序
        if (is_build_map) {
            // std::cout << bodyHeight << "RECV: " << RecvHighROS.bodyHeight << std::endl;
            // 如果检测到 遥控器 X 键的相应变化则进入保存点的程序
            if (RecvHighROS.wirelessRemote[3]-last_set_cmd==4) {
                // 判断巡逻点文件是否打开
                if (patrol_points_file.is_open()) {
                    try {
                        transformStamped = tfBuffer.lookupTransform("map", "base_link",
                                                                    ros::Time(0));
                    }
                    catch (tf2::TransformException &ex) {
                        ROS_WARN("%s",ex.what());
                        ros::Duration(1.0).sleep();
                        continue;
                    }
                    // 获取狗当前 yaw 的信息
                    yaw = tf2::getYaw(transformStamped.transform.rotation);
                    // 如果保存的点和上一个巡逻点的距离太近则不保存新的点（x y 小于 3cm 且角度 yaw 小于 0.09 弧度）
                    if (transformStamped.transform.translation.x-last_point[0]<=0.03) {
                        if (transformStamped.transform.translation.y-last_point[1]<=0.03) {
                            if (yaw-last_point[2]<=0.09) {
                                is_same_point = true;
                            }
                        }
                    }
                    // 如果是同一个点则不保存，否则就保存
                    if (is_same_point) {
                        std::cout << "Same points, not saved !!!!!!!!!!!!!!!!!!" << std::endl;
                    } else {
                        patrol_points_file << transformStamped.transform.translation.x 
                                        << " "
                                        << transformStamped.transform.translation.y
                                        << " "
                                        << yaw
                                        << " "
                                        << 3 << std::endl;  // stay time, sec
                        std::cout << "Save points !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
                        std::cout << transformStamped.transform.translation.x 
                                << " "
                                << transformStamped.transform.translation.y
                                << " "
                                << yaw
                                << " "
                                << 3 << std::endl;
                        last_point[0] = transformStamped.transform.translation.x;
                        last_point[1] = transformStamped.transform.translation.y;
                        last_point[2] = yaw;
                    }
                    // 将判断归位 方便下一次判断
                    is_same_point=false;
                }
            }
        }
        // 保存上一次的键值，以便判断遥控器状态
        last_set_cmd = RecvHighROS.wirelessRemote[3];
        // 如果遥控器表示机器狗可以运动，则发布狗相应的运动状态
        if (can_move) {
            SendHighROS.mode = mode;
        } else {
            SendHighROS.mode = 1;
        }
        // 设置狗的运动信息
        SendHighROS.forwardSpeed = linear_x;
        SendHighROS.sideSpeed = linear_y;
        SendHighROS.rotateSpeed = angular_z;
        SendHighROS.roll  = 0;
        SendHighROS.pitch = 0;
        SendHighROS.yaw = 0;

        // 编码 ros 的运动信息到 底层需要的 格式
        SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
        roslcm.Send(SendHighLCM);
        // ros 进入回调函数
        ros::spinOnce();
        // 以一定的时间暂停，以达到设置的循环频率
        loop_rate.sleep(); 
    }
    // 关闭巡逻点文件
    patrol_points_file.close();

    return 0;
}

int main(int argc, char *argv[]){
    // ros 初始化
    ros::init(argc, argv, "dog_control_node");
    // 设置 机器狗通信的固件版本
    std::string firmwork;
    ros::param::get("/firmwork", firmwork);

    // 使用 3_2 或者 3_1
    if(firmwork == "3_2"){
        std::string robot_name;
        UNITREE_LEGGED_SDK::LeggedType rname;
        ros::param::get("/robot_name", robot_name);
        if(strcasecmp(robot_name.c_str(), "A1") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::A1;
        else if(strcasecmp(robot_name.c_str(), "Aliengo") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::Aliengo;

        UNITREE_LEGGED_SDK::InitEnvironment();
        UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
        mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
    }
    
}
