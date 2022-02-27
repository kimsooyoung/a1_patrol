/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "convert.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <pthread.h>
#include <ros/ros.h>
#include <string>

#include <chrono>
#include <thread>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>

uint8_t mode = 1;
uint8_t a1_mode = 1;

float linear_x = 0.0;
float linear_y = 0.0;
float angular_z = 0.0;

bool can_move = false;
bool is_build_map = false;

float yaw = 0;

sensor_msgs::Imu dogImu;
nav_msgs::Odometry dogOdom;
std_msgs::Bool dog_can_move;

void cmd_vel_cb(const geometry_msgs::Twist &msg)
{
  linear_x = msg.linear.x;
  linear_y = msg.linear.y;
  angular_z = msg.angular.z;

  std::cout << linear_x << "," << linear_y << "," << angular_z << std::endl;
}

void mode_cb(const std_msgs::UInt8 &msg)
{
  mode = msg.data;

  std::cout << int(mode) << std::endl;
  std::cout << "flag" << std::endl;
}

void a1_mode_cb(const std_msgs::UInt8 &msg)
{
  a1_mode = msg.data;

  std::cout << int(a1_mode) << std::endl;
  std::cout << "a1_mode_cb flag" << std::endl;
}

template <typename TLCM>
void *update_loop(void *param)
{
  TLCM *data = (TLCM *)param;
  while (ros::ok)
  {
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
  std::cin.ignore();

  int last_start_cmd = 0;
  int last_set_cmd = 0;
  long motiontime = 0;

  // ros 구문 자루와 인자 설정
  ros::NodeHandle n;
  ros::Rate loop_rate(500);

  ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 1, cmd_vel_cb);
  ros::Subscriber mode_sub = n.subscribe("/dog_mode", 1, mode_cb);
  ros::Subscriber a1_mode_sub = n.subscribe("/a1_mode", 1, a1_mode_cb);
  ros::Publisher dog_imu_pub = n.advertise<sensor_msgs::Imu>("/imu_raw", 1000);
  ros::Publisher dog_can_move_pub = n.advertise<std_msgs::Bool>("/dog_can_move", 1000);
  ros::Publisher dog_odom_pub = n.advertise<nav_msgs::Odometry>("/dog_odom", 1000);

  TCmd SendHighLCM = {0};
  TState RecvHighLCM = {0};
  unitree_legged_msgs::HighCmd SendHighROS;
  unitree_legged_msgs::HighState RecvHighROS;

  roslcm.SubscribeState();

  pthread_t tid;
  pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

  while (ros::ok())
  {
    roslcm.Get(RecvHighLCM);
    RecvHighROS = ToRos(RecvHighLCM);

    // 리모컨의 start 키를 통해 개의 능동 여부를 제어합니다.
    if (RecvHighROS.wirelessRemote[2] - last_start_cmd == 4)
    {
      can_move = !can_move;
      if (can_move)
      {
        mode = 2;
      }
      std::cout << "!!!!! can move: " << bool(can_move) << std::endl;
    }

    dog_can_move.data = can_move;
    dog_can_move_pub.publish(dog_can_move);
    // std::cout << int(RecvHighROS.wirelessRemote[2])  << " mode: " << int(mode) << std::endl;
    last_start_cmd = RecvHighROS.wirelessRemote[2];

    // Prepare IMU MSG
    dogImu.header.frame_id = "imu_link";
    dogImu.header.stamp = ros::Time::now();
    // dog's orientation 개의 방향 정보
    dogImu.orientation.w = RecvHighROS.imu.quaternion[0];
    dogImu.orientation.x = -RecvHighROS.imu.quaternion[1];
    dogImu.orientation.y = -RecvHighROS.imu.quaternion[2];
    dogImu.orientation.z = -RecvHighROS.imu.quaternion[3];
    // dogImu.orientation_covariance[0] = -1;
    // dog's angular_velocity 개의 각속도 정보
    dogImu.angular_velocity.x = RecvHighROS.imu.gyroscope[0];
    dogImu.angular_velocity.y = RecvHighROS.imu.gyroscope[1];
    dogImu.angular_velocity.z = RecvHighROS.imu.gyroscope[2];
    // dog's linear_acceleration
    dogImu.linear_acceleration.x = RecvHighROS.imu.accelerometer[0];
    dogImu.linear_acceleration.y = RecvHighROS.imu.accelerometer[1];
    dogImu.linear_acceleration.z = RecvHighROS.imu.accelerometer[2];
    // publish dog's imu message
    dog_imu_pub.publish(dogImu);

    // Prepare Odom MSG
    dogOdom.header.frame_id = "odom";
    dogOdom.child_frame_id = "base_footprint";
    dogOdom.header.stamp = ros::Time::now();

    dogOdom.pose.pose.position.x = RecvHighROS.position[0];
    dogOdom.pose.pose.position.y = RecvHighROS.position[1];
    dogOdom.pose.pose.orientation = dogImu.orientation;
    dogOdom.pose.covariance.fill(0.0);

    dogOdom.twist.twist.linear.x = RecvHighROS.velocity[0];
    dogOdom.twist.twist.linear.y = RecvHighROS.velocity[1];
    // TODO : Check difference btw belows
    dogOdom.twist.twist.angular.z = RecvHighROS.velocity[2];
    // dogOdom.twist.twist.angular.z = RecvHighROS.yawSpeed;
    dogOdom.twist.covariance.fill(0.0);
    dog_odom_pub.publish(dogOdom);

    // 리모컨 상태를 판단하기 위해 이전 키 값을 저장합니다.
    last_set_cmd = RecvHighROS.wirelessRemote[3];
    // 만약 리모컨이 로봇 개가 운동할 수 있음을 나타낸다면, 개의 운동 상태를 알린다.
    if (can_move)
    {
      SendHighROS.mode = mode;
    }
    else
    {
      SendHighROS.mode = 1;
    }

    // 개의 운동 정보 설정
    switch (a1_mode)
    {
    case 1:
      motiontime = 0;
      SendHighROS.mode = 1;
      SendHighROS.velocity[0] = 0.0;
      SendHighROS.velocity[1] = 0.0;
      SendHighROS.yawSpeed = angular_z;
      SendHighROS.euler[0] = linear_y * 0.7;
      SendHighROS.euler[1] = linear_x * 0.5;
      SendHighROS.euler[2] = angular_z * 0.3;
      break;
    case 2:
      motiontime = 0;
      SendHighROS.mode = 2;
      SendHighROS.gaitType = 1;
      SendHighROS.velocity[0] = linear_x;
      SendHighROS.velocity[1] = linear_y;
      // Check required
      // SendHighROS.velocity[2] = angular_z;
      SendHighROS.yawSpeed = angular_z;

      SendHighROS.bodyHeight = 0.1;
      SendHighROS.footRaiseHeight = 0.1;
      break;
    case 3:
      motiontime = 0;
      SendHighROS.mode = 2;
      SendHighROS.gaitType = 2;
      SendHighROS.velocity[0] = linear_x;
      SendHighROS.velocity[1] = linear_y;
      // Check required
      // SendHighROS.velocity[2] = angular_z;
      SendHighROS.yawSpeed = angular_z;

      SendHighROS.bodyHeight = 0.1;
      SendHighROS.footRaiseHeight = 0.1;
      break;
    case 4:
      if (motiontime < 5001){
        motiontime = motiontime+2;
        if (motiontime % 10 == 0)
          std::cout << motiontime << std::endl;
      }

      if (motiontime > 0 && motiontime < 2000){
        SendHighROS.mode = 5;
      }
      if (motiontime > 2000 && motiontime < 4000){
        SendHighROS.mode = 6;
      }
      if (motiontime > 4000 && motiontime < 5000){
        SendHighROS.mode = 0;
      }
      if (motiontime > 5000) {
        SendHighROS.mode = 2;
        SendHighROS.gaitType = 3;
        SendHighROS.velocity[0] = linear_x;
        SendHighROS.velocity[1] = linear_y;
        // Check required
        // SendHighROS.velocity[2] = angular_z;
        SendHighROS.yawSpeed = angular_z;

        SendHighROS.bodyHeight = 0.1;
        SendHighROS.footRaiseHeight = 0.1;
      }
      break;
    default:
      motiontime = 0;
      break;
    }

    // ros의 움직임 정보를 하위 단계까지 인코딩하는 데 필요한 형식
    SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
    roslcm.Send(SendHighLCM);

    // ros 리플레이 함수 진입
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

int main(int argc, char *argv[])
{
  // ros 초기화
  ros::init(argc, argv, "dog_control_node");
  // 로봇 개 통신의 펌웨어 버전을 설정하다.
  std::string firmwork;
  ros::param::get("/firmwork", firmwork);
  // nh.param<std::string>("firmwork", firmwork, "3_2");

  // // 3_2 또는 3_1 사용
  // if(firmwork == "3_2"){

  // }

  std::string robot_name;
  UNITREE_LEGGED_SDK::LeggedType rname;
  ros::param::get("/robot_name", robot_name);
  if (strcasecmp(robot_name.c_str(), "A1") == 0)
    rname = UNITREE_LEGGED_SDK::LeggedType::A1;
  else if (strcasecmp(robot_name.c_str(), "Aliengo") == 0)
    rname = UNITREE_LEGGED_SDK::LeggedType::Aliengo;

//   mainHelper();

  // UNITREE_LEGGED_SDK::InitEnvironment();
  UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
  mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
}