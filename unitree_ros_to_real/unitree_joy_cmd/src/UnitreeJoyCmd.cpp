/*********************************************************************
 * Software License
 *  Copyright (c) 2022, WeGo Robotices Co., Ltd. && Road Balance Co. All Rights Reserved.
 *********************************************************************/

#include "unitree_joy_cmd/UnitreeJoyCmd.hpp"

UnitreeJoyCmd::UnitreeJoyCmd(ros::NodeHandle *nh)
{
    /// Publisher, Subscriber, Timer 생성
    unitree_cmd_pub_ = nh->advertise<unitree_legged_msgs::HighCmd>("untiree_quadrupped_high_cmd", 10);
    joy_sub_ = nh->subscribe("joy", 10, &UnitreeJoyCmd::subCallback, this);
    control_timer_ = nh->createTimer(ros::Duration(0.02), &UnitreeJoyCmd::timerCallback, this);

    /// Parameter 설정을 위해 추가 NodeHandler 생성
    ros::NodeHandle nh_private("~");

    /// Parameter 설정 및 초기화
    nh_private.param<float>("roll_gain", roll_gain_, -0.8);
    // 부호 상쇄
    roll_gain_ *= -1;
    ROS_INFO("roll_gain : %f", roll_gain_);

    nh_private.param<float>("pitch_gain", pitch_gain_, 0.5);
    ROS_INFO("pitch_gain : %f", pitch_gain_);

    nh_private.param<float>("yaw_gain", yaw_gain_, 0.3);
    ROS_INFO("yaw_gain : %f", yaw_gain_);

    nh_private.param<float>("bodyheight_gain", bodyheight_gain_, 0.2);
    ROS_INFO("bodyheight_gain : %f", bodyheight_gain_);
}

void UnitreeJoyCmd::timerCallback(const ros::TimerEvent &event)
{
    unitree_cmd_pub_.publish(unitree_cmd_msg_);
}

void UnitreeJoyCmd::subCallback(const sensor_msgs::Joy &data)
{
    // Assume JoyStick is on "X" mode
    joy_keys.left_updown = data.axes[1];
    joy_keys.left_leftright = data.axes[0];
    joy_keys.right_updown = data.axes[4];
    joy_keys.right_leftright = data.axes[3];

    // +1 btn up / -1 btn down
    joy_keys.btn_updown = data.axes[7];
    // +1 btn left / -1 btn right
    joy_keys.btn_leftright = data.axes[6];

    joy_keys.btn_a = isTrue(data.buttons[0]);
    joy_keys.btn_b = isTrue(data.buttons[1]);
    joy_keys.btn_x = isTrue(data.buttons[2]);
    joy_keys.btn_y = isTrue(data.buttons[3]);

    joy_keys.btn_LB = isTrue(data.buttons[4]);
    joy_keys.btn_RB = isTrue(data.buttons[5]);

    joy_keys.btn_back = isTrue(data.buttons[6]);
    joy_keys.btn_start = isTrue(data.buttons[7]);

    parseControlMode();

    /// X mode의 경우 roll, pitch, yaw, bodyHeight 값만을 파싱함
    if (contorl_mode_ == ContorlMode::X_MODE)
    {
        parsePoseMode();
    }
    else
    {
        parseBodyHeight();
        parseFootRaiseHeight();
        parsePositionMode();
    }
    
    /// 조이스틱 키값 갱신
    prev_joy_keys = joy_keys;
}

bool UnitreeJoyCmd::parsePoseMode()
{
    unitree_cmd_msg_.mode = 1;

    unitree_cmd_msg_.velocity[0] = 0.0;
    unitree_cmd_msg_.velocity[1] = 0.0;
    unitree_cmd_msg_.yawSpeed = 0.0;

    unitree_cmd_msg_.euler[0] = joy_keys.left_leftright * -0.8;
    unitree_cmd_msg_.euler[1] = joy_keys.left_updown * 0.5;
    unitree_cmd_msg_.euler[2] = joy_keys.right_leftright * 0.3;
    unitree_cmd_msg_.bodyHeight = joy_keys.right_updown * 0.2;

    return true;
}

bool UnitreeJoyCmd::parsePositionMode()
{
    unitree_cmd_msg_.mode = 2;

    unitree_cmd_msg_.velocity[0] = joy_keys.left_updown;
    unitree_cmd_msg_.velocity[1] = joy_keys.left_leftright;

    unitree_cmd_msg_.yawSpeed = joy_keys.right_leftright * 2;

    switch (contorl_mode_)
    {
    case ContorlMode::A_MODE:
        unitree_cmd_msg_.gaitType = 1;
        break;
    case ContorlMode::B_MODE:
        unitree_cmd_msg_.gaitType = 2;
        break;
    case ContorlMode::Y_MODE:
        unitree_cmd_msg_.gaitType = 3;
        break;
    }

    return true;
}

bool UnitreeJoyCmd::parseControlMode()
{
    if (joy_keys.btn_x)
        contorl_mode_ = ContorlMode::X_MODE;
    if (joy_keys.btn_a)
        contorl_mode_ = ContorlMode::A_MODE;
    if (joy_keys.btn_b)
        contorl_mode_ = ContorlMode::B_MODE;
    if (joy_keys.btn_y)
        contorl_mode_ = ContorlMode::Y_MODE;

    return true;
}

bool UnitreeJoyCmd::parseBodyHeight()
{
    /// floating point 연산을 위해 특정한 값 사용
    if (joy_keys.btn_updown == 1.0 && prev_joy_keys.btn_updown == 0.0)
        unitree_cmd_msg_.bodyHeight += 0.03125f;
    if (joy_keys.btn_updown == -1.0 && prev_joy_keys.btn_updown == 0.0)
        unitree_cmd_msg_.bodyHeight -= 0.03125f;

    return true;
}

bool UnitreeJoyCmd::parseFootRaiseHeight()
{
    /// floating point 연산을 위해 특정한 값 사용
    if (joy_keys.btn_LB && !prev_joy_keys.btn_LB)
        unitree_cmd_msg_.footRaiseHeight -= 0.0625f;
    if (joy_keys.btn_RB && !prev_joy_keys.btn_RB)
        unitree_cmd_msg_.footRaiseHeight += 0.0625f;

    return true;
}