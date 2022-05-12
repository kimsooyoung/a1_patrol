/*********************************************************************
 * Software License
 *  Copyright (c) 2022, WeGo Robotices Co., Ltd. && Road Balance Co. All Rights Reserved.
 *********************************************************************/

#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <tf2/utils.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
// #include <unitree_legged_msgs/BmsState.h>
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

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

/**
 * @brief Unitree Legged Robot HighLevel Control을 위한 Template Class
 *
 * @tparam TCmd SDK Cmd Type
 * @tparam TState SDK State Type
 * @tparam TLCM SDK LCM Type
 */
template <typename TCmd, typename TState, typename TLCM>
class HighLevelHelper
{
private:
  /// SDK Type DB
  TCmd SendHighLCM_ = {0};
  TState RecvHighLCM_ = {0};
  TLCM roslcm_;

  /// pthread id
  pthread_t tid_;

  /// SDK DB와 동일한 요소를 갖진 ROS msg들
  unitree_legged_msgs::HighState RecvHighROS_;
  unitree_legged_msgs::HighCmd SendHighROS_;

  /// 일정 시간 제어 데이터 수신이 없다면 LCM 서버가 중단됨
  /// 이에 따라 LCM/ROS가 주기적인 Pub/Sub이 이루어짐
  ros::Timer timer_;

  /// ROS Publisher, Subscriber
  ros::Publisher imu_pub_;
  ros::Publisher movable_pub_;
  ros::Publisher odom_pub_;

  ros::Subscriber control_sub_;

  /// HighState 데이터를 여러 topic으로 쪼갠다.
  sensor_msgs::Imu dog_imu_;
  nav_msgs::Odometry dog_odom_;
  std_msgs::Bool dog_can_move_;

  /// tf 관련 변수들
  /// imu tf, odom tf를 생성하며 둘은 같은 좌표를 갖는다.
  tf::Quaternion q_raw_, q_offset_;

  tf::Transform imu_transform_;
  tf::Transform odom_transform_;

  tf::TransformBroadcaster imu_broadcaster_;
  tf::TransformBroadcaster odom_broadcaster_;

  // ROS Parameters
  bool publish_tf_;
  bool verbose_;
  std::string imu_link_id_, base_link_id_, odom_link_id_;

public:
  /**
   * @brief Construct a new High Level Helper object
   * ROS topic Sub/Pub을 위한 초기 작업 실행
   * Timer 설정
   * 각종 parameter 설정 및 초기화
   * LCM <> ROS간 통신을 위한 준비
   * 
   * @param nh Class 생성 시 main으로 부터 NodeHandler를 전달받음
   * @param roslcm LCM Server와 통신하기 위한 LCM Client + ROS 매개체
   */
  HighLevelHelper(ros::NodeHandle *nh, TLCM &roslcm) : roslcm_(roslcm)
  {
    /// LCM 상태 변경
    roslcm_.SubscribeState();

    /// thread 생성, join 및 close는 이 코드에서 실행되지 않음에 주의한다.
    pthread_create(&tid_, NULL, update_loop<TLCM>, &roslcm_);

    /// ROS 관련 초기화 설정 
    timer_ = nh->createTimer(ros::Duration(0.002), &HighLevelHelper::timerCallback, this);

    imu_pub_ = nh->advertise<sensor_msgs::Imu>("imu_raw", 10);
    movable_pub_ = nh->advertise<std_msgs::Bool>("dog_can_move_", 10);
    odom_pub_ = nh->advertise<nav_msgs::Odometry>("dog_odom", 10);

    control_sub_ = nh->subscribe("untiree_quadrupped_high_cmd", 10, &HighLevelHelper::controlSubCallback, this);

    /// Parameter 설정을 위해 추가 NodeHandler 생성
    ros::NodeHandle nh_private("~");

    nh_private.param<std::string>("imu_link", imu_link_id_, "imu_link");
    ROS_INFO("imu_link_id : %s", imu_link_id_.c_str());

    nh_private.param<std::string>("base_link", base_link_id_, "base_link");
    ROS_INFO("base_link_id : %s", base_link_id_.c_str());

    nh_private.param<std::string>("odom_link", odom_link_id_, "odom");
    ROS_INFO("odom_link_id : %s", odom_link_id_.c_str());

    nh_private.param<bool>("publish_tf", publish_tf_, true);
    ROS_INFO("publish_tf : %s", publish_tf_ ? "true" : "false");

    nh_private.param<bool>("verbose", verbose_, false);
    ROS_INFO("verbose : %s", verbose_ ? "true" : "false");

    // imu 오차 발생 시 보정을 위한 offset
    q_offset_.setRPY(0, 0, 1.5707);
  }

  /**
   * @brief unitree_legged_msgs::HighCmd를 다시 한 번 파싱한다.
   * TODO : 예외처리 로직 추가하기 
   * ex) mode가 9로 잘못 변경되는 경우, 갑자기 백덤블링을 할 수 있음
   * 
   * @param high_cmd unitree_legged_msgs::HighCmd 타입 Subscribe msg
   */
  void controlSubCallback(const unitree_legged_msgs::HighCmd &high_cmd)
  {
    SendHighROS_.mode = high_cmd.mode;

    SendHighROS_.gaitType = high_cmd.gaitType;

    SendHighROS_.euler[0] = high_cmd.euler[0];
    SendHighROS_.euler[1] = high_cmd.euler[1];
    SendHighROS_.euler[2] = high_cmd.euler[2];

    SendHighROS_.velocity[0] = high_cmd.velocity[0];
    SendHighROS_.velocity[1] = high_cmd.velocity[1];

    SendHighROS_.yawSpeed = high_cmd.yawSpeed;

    SendHighROS_.bodyHeight = high_cmd.bodyHeight;
    SendHighROS_.footRaiseHeight = high_cmd.footRaiseHeight;
  }

  /**
   * @brief 50Hz 주기로 실행되는 Timer 
   * LCM State msg를 받아 ROS 형태로 반환하고, 반환된 데이터는 tf등 각종 유의미한 유틸리티로 변환된다. 
   * 동시에 ROS msg를 LCM Control msg로 반환하여 server로 전달하는 역할도 수행한다.
   * 
   * @param event Timer 사용을 위해 필수로 추가되는 매개변수 
   */
  void timerCallback(const ros::TimerEvent &event)
  {
    roslcm_.Get(RecvHighLCM_);
    /// LCM State to ROS msg
    RecvHighROS_ = ToRos(RecvHighLCM_);

    /// 갱신된 데이터들을 나누어 tf update, topic publish 등에 사용한다.
    parseCanMove();
    parseIMU();
    parseOrientation();
    parseOdom();

    paseTest();

    /// topic publish 
    movable_pub_.publish(dog_can_move_);
    imu_pub_.publish(dog_imu_);
    odom_pub_.publish(dog_odom_);

    /// tf publish
    if (publish_tf_)
    {
      publishImuTF();
      publishOdomTF();
    }

    if (verbose_)
      printEulerAngle();

    /// ROS Control msg to LCM Control msg
    SendHighLCM_ = ToLcm(SendHighROS_, SendHighLCM_);
    roslcm_.Send(SendHighLCM_);
  }

  /**
   * @brief 로봇의 조종 가능 여부를 확인한다.
   * 
   * @return true 조종 가능
   * @return false 조종 불가능
   */
  bool parseCanMove()
  {
    /// 리모컨의 start키 입력 수신 여부에 따라 로봇의 작동 가능 여부를 확인한다.
    static uint8_t print_count = 0;
    if (RecvHighLCM_.mode == 2)
    {
      dog_can_move_.data = true;
      if (print_count == 0)
      {
        ROS_INFO("robot can move: %s", dog_can_move_.data ? "true" : "false");
        print_count++;
      }
    }
    else
    {
      dog_can_move_.data = false;
      print_count = 0;
    }

    return true;
  }

  /**
   * @brief RecvHighROS_ 중 imu 데이터만을 추출하여 topic msg를 구성한다.
   * 
   * @return true 파싱 성공
   * @return false 파싱 실패 (TODO)
   */
  bool parseIMU()
  {
    // Prepare IMU MSG
    dog_imu_.header.frame_id = imu_link_id_;
    dog_imu_.header.stamp = ros::Time::now();

    // 방향 데이터
    dog_imu_.orientation.x = RecvHighROS_.imu.quaternion[0];
    dog_imu_.orientation.y = RecvHighROS_.imu.quaternion[1];
    dog_imu_.orientation.z = RecvHighROS_.imu.quaternion[2];
    dog_imu_.orientation.w = RecvHighROS_.imu.quaternion[3];

    // 각속도 데이터
    dog_imu_.angular_velocity.x = RecvHighROS_.imu.gyroscope[0];
    dog_imu_.angular_velocity.y = RecvHighROS_.imu.gyroscope[1];
    dog_imu_.angular_velocity.z = RecvHighROS_.imu.gyroscope[2];

    // 가속도 데이터
    dog_imu_.linear_acceleration.x = RecvHighROS_.imu.accelerometer[0];
    dog_imu_.linear_acceleration.y = RecvHighROS_.imu.accelerometer[1];
    dog_imu_.linear_acceleration.z = RecvHighROS_.imu.accelerometer[2];

    return true;
  }

  /**
   * @brief RecvHighROS_ 중 각도 데이터만을 추출한 뒤 오일러 각도 변환.
   * 이것은 이후 디버깅 및 tf publish 시 사용하게 된다.
   * 
   * @return true 파싱 성공
   * @return false 파싱 실패 (TODO)
   */
  bool parseOrientation()
  {
    q_raw_[0] = RecvHighROS_.imu.quaternion[0];
    q_raw_[1] = RecvHighROS_.imu.quaternion[1];
    q_raw_[2] = RecvHighROS_.imu.quaternion[2];
    q_raw_[3] = RecvHighROS_.imu.quaternion[3];

    return true;
  }

  /**
   * @brief RecvHighROS_ 중 odom과 관련된 데이터만을 추출하여 topic msg를 구성한다.
   * 
   * @return true 파싱 성공
   * @return false 파싱 실패 (TODO)
   */
  bool parseOdom()
  {
    // Prepare Odom MSG
    dog_odom_.header.frame_id = odom_link_id_;
    dog_odom_.child_frame_id = base_link_id_;
    dog_odom_.header.stamp = ros::Time::now();

    // 위치 데이터 - 굴곡진 환경에서 큰 오차를 보임
    dog_odom_.pose.pose.position.x = RecvHighROS_.position[0];
    dog_odom_.pose.pose.position.y = RecvHighROS_.position[1];
    dog_odom_.pose.pose.orientation = dog_imu_.orientation;
    dog_odom_.pose.covariance.fill(0.0);

    // 속도 데이터
    dog_odom_.twist.twist.linear.x = RecvHighROS_.velocity[0];
    dog_odom_.twist.twist.linear.y = RecvHighROS_.velocity[1];
    dog_odom_.twist.twist.angular.z = RecvHighROS_.yawSpeed;
    dog_odom_.twist.covariance.fill(0.0);

    return true;
  }

  bool paseTest()
  {
    // Cartesian[4] footPosition2Body      # foot position relative to body
    // Cartesian[4] footSpeed2Body         # foot speed relative to body
    unitree_legged_msgs::Cartesian footPosition2Body[4];
    unitree_legged_msgs::Cartesian footSpeed2Body[4];

    // uint8[40] wirelessRemote
    int8_t wirelessRemote[40];

    for(int i = 0; i < 4; i++){
      footPosition2Body[i] = RecvHighROS_.footPosition2Body[i];
      footSpeed2Body[i] = RecvHighROS_.footSpeed2Body[i];
    }

    std::cout << "footPosition2Body " << std::endl;
    for(int i = 0; i < 4; i++)
      std::cout << footPosition2Body[i] << " ";
    std::cout << std::endl;

    std::cout << "footSpeed2Body " << std::endl;
    for(int i = 0; i < 4; i++)
      std::cout << footSpeed2Body[i] << " ";
    std::cout << std::endl;
  }

  /**
   * @brief imu_link를 이름으로 갖는 tf frame publish
   * 현재는 imu_link와 base_link가 일치한다.
   * 
   * @return true publish 성공
   * @return false publish 실패
   */
  bool publishImuTF()
  {

    tf::Quaternion unit_q;
    unit_q[0] = 0.0;
    unit_q[1] = 0.0;
    unit_q[2] = 0.0;
    unit_q[3] = 1.0;

    imu_transform_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    imu_transform_.setRotation(unit_q);
    imu_broadcaster_.sendTransform(tf::StampedTransform(
        imu_transform_, ros::Time::now(), base_link_id_, imu_link_id_));

    return true;
  }

  /**
   * @brief odom_link_id_를 이름으로 갖는 odom tf frame publish
   * 해당 odom은 unitree 로봇 내에서 자체적으로 계산된 값으로
   * 굴곡진 지형을 지나거나, 급작스런 속도 변화 발생 시 odom이 불안정해진다.
   * 자체적으로 사용하기보다 sensor fusion을 거친 뒤 사용하기를 추천
   * 
   * @return true publish 성공
   * @return false publish 실패
   */
  bool publishOdomTF()
  {

    odom_transform_.setOrigin(tf::Vector3(dog_odom_.pose.pose.position.x, dog_odom_.pose.pose.position.y, 0.0));
    odom_transform_.setRotation(q_raw_);
    odom_broadcaster_.sendTransform(tf::StampedTransform(
        odom_transform_, ros::Time::now(), odom_link_id_, base_link_id_));

    return true;
  }

  /**
   * @brief 디버깅용 오일러 각도 출력
   * 
   * @return true 
   * @return false 
   */
  bool printEulerAngle()
  {

    tf::Matrix3x3 euler_mat(q_raw_);

    double roll, pitch, yaw;
    euler_mat.getRPY(roll, pitch, yaw);

    ROS_INFO("\nroll : %f \npitch : %f \nyaw : %f", roll, pitch, yaw);

    return true;
  }

  /**
   * @brief Destroy the High Level Helper object
   * 
   */
  ~HighLevelHelper()
  {
  }
};
