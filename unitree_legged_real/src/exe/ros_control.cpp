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

// 매개변수 설정
float linear_x = 0.0;
float linear_y = 0.0;
float angular_z = 0.0;
uint8_t mode = 1;
bool can_move = false;
bool is_build_map = false;
float yaw = 0;

// 개 imu의 데이터를 만듭니다.
// dog's imu message to publish
sensor_msgs::Imu dogImu;
std_msgs::Bool dog_can_move;

// ros에서 움직임 명령어를 읽어오는 callback 함수
void cmd_vel_cb(const geometry_msgs::Twist& msg){
    linear_x = msg.linear.x;
    linear_y = msg.linear.y;
    angular_z = msg.angular.z;
}

// ros에서 움직임 명령어를 읽어오는 callback 함수
void mode_cb(const std_msgs::UInt8& msg){
    mode = msg.data;
}

// LCM template
template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

// main 함수 template
template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();
    
    // ros 매개변수 가져오기
    ros::param::get("dog_control_node/is_build_map", is_build_map);
    // 인자순찰점 파일의 기본값 설정
    std::string patrol_points_file_path = "";
    // 설계도 작업인지 아닌지를 판단해야 하며, 그렇지 않으면 순찰점을 보존할 필요가 없다.
    if (is_build_map) {
        ros::param::get("dog_control_node/patrol_points_file", patrol_points_file_path);
    }
    // 순찰점 파일을 읽다.
    std::ofstream patrol_points_file(patrol_points_file_path, std::ios::trunc);
    int last_start_cmd = 0;
    int last_set_cmd = 0;
    float last_point[3] = {0};
    bool is_same_point = false;

    // 좌표 변환 변수 만들기
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;

    // ros 구문 자루와 인자 설정
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

    // 새 스레드 열기
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
        // 리모컨의 start 키를 통해 개의 능동 여부를 제어합니다.
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
        // 만약 렌더링 과정이라면, 순찰점을 저장하는 프로그램을 수행합니다.
        if (is_build_map) {
            // std::cout << bodyHeight << "RECV: " << RecvHighROS.bodyHeight << std::endl;
            // 리모컨 X키의 해당 변화가 감지되면 저장소에 들어가는 프로그램
            if (RecvHighROS.wirelessRemote[3]-last_set_cmd==4) {
                // 순찰점 파일 열림 여부 판단
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
                    // 개의 현재 yaw 정보 가져오기
                    yaw = tf2::getYaw(transformStamped.transform.rotation);
                    // 보존된 지점과 이전 초계점의 거리가 너무 가까울 경우.
                    // 새로운 점(x y 3cm 이하, 각도 yaw 0.09호 이하)은 보존되지 않습니다.
                    if (transformStamped.transform.translation.x-last_point[0]<=0.03) {
                        if (transformStamped.transform.translation.y-last_point[1]<=0.03) {
                            if (yaw-last_point[2]<=0.09) {
                                is_same_point = true;
                            }
                        }
                    }
                    // 같은 점이라면 저장하지 않습니다. 그렇지 않으면 저장합니다.
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
                    // 판단을 귀순시켜 다음 판단을 용이하게 하다.
                    is_same_point=false;
                }
            }
        }
        // 리모컨 상태를 판단하기 위해 이전 키 값을 저장합니다.
        last_set_cmd = RecvHighROS.wirelessRemote[3];
        // 만약 리모컨이 로봇 개가 운동할 수 있음을 나타낸다면, 개의 운동 상태를 알린다.
        if (can_move) {
            SendHighROS.mode = mode;
        } else {
            SendHighROS.mode = 1;
        }
        // 개의 운동 정보 설정
        SendHighROS.velocity[0] = linear_x;
        SendHighROS.velocity[1] = linear_y;
        SendHighROS.yawSpeed    = angular_z;
        SendHighROS.roll  = 0;
        SendHighROS.pitch = 0;
        SendHighROS.yaw = 0;

        // ros의 움직임 정보를 하위 단계까지 인코딩하는 데 필요한 형식
        SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
        roslcm.Send(SendHighLCM);
        // ros 리플레이 함수 진입
        ros::spinOnce();
        // 설정된 순환 빈도에 도달하기 위해 일정한 시간으로 일시 정지한다.
        loop_rate.sleep(); 
    }
    // 순찰점 문건을 폐쇄하다.
    patrol_points_file.close();

    return 0;
}

int main(int argc, char *argv[]){
    // ros 초기화
    ros::init(argc, argv, "dog_control_node");
    // 로봇 개 통신의 펌웨어 버전을 설정하다.
    std::string firmwork;
    ros::param::get("/firmwork", firmwork);

    // 3__2 또는 3_1 사용
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
