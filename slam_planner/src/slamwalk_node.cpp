/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include <unitree_legged_sdk/unitree_legged_sdk.h>
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

#include <ros/ros.h>
#include <pthread.h>
#include <geometry_msgs/Twist.h>


using namespace UNITREE_LEGGED_SDK;

float forwardSpeed = 0.0;
float rotateSpeed = 0.0;

class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::A1), udp(8910, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)){
        udp.InitCmdData(cmd);
        udp.SetDisconnectTime(dt, 1);
        // udp.SetDisconnectTime(0, 0);
    }
    // void UDPRecv();
    // void UDPSend();
    // void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    // int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};



void cmd_callback(const geometry_msgs::Twist& cmd_vel)
{
	ROS_INFO("Linear Components:[%f,%f,%f]", cmd_vel.linear.x,  cmd_vel.linear.y,  cmd_vel.linear.z);
	ROS_INFO("Angular Components:[%f,%f,%f]",cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z);

	forwardSpeed = cmd_vel.linear.x;
	rotateSpeed = cmd_vel.angular.z;
}


int main(int argc, char** argv) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl;
	
    ros::init(argc, argv, "slamwalk_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/cmd_vel", 1, cmd_callback);
    ros::Rate loop_rate(500);

    Custom custom(HIGHLEVEL);
    custom.cmd.mode = 0;
    custom.cmd.gaitType = 0;
    custom.cmd.speedLevel = 0;
    custom.cmd.footRaiseHeight = 0;
    custom.cmd.bodyHeight = 0;
    custom.cmd.euler[0]  = 0;
    custom.cmd.euler[1] = 0;
    custom.cmd.euler[2] = 0;
    custom.cmd.velocity[0] = 0.0f;
    custom.cmd.velocity[1] = 0.0f;
    custom.cmd.yawSpeed = 0.0f;

    InitEnvironment();

    // LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    // LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    // LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    // loop_udpSend.start();
    // loop_udpRecv.start();
    // loop_control.start();
	int count=0;
    while(ros::ok()){
        // udp.GetRecv(state);
        if(forwardSpeed != 0 || rotateSpeed !=0){
            custom.cmd.mode = 2;
            // cmd.gaitType = 1;
            custom.cmd.velocity[0] = forwardSpeed; // -1  ~ +1
            custom.cmd.velocity[1] = 0; 
            custom.cmd.yawSpeed = rotateSpeed;
            // cmd.footRaiseHeight = 0;
	    custom.udp.SetSend(custom.cmd);
	    custom.udp.Send();
	    count = 0;
        }else{
            custom.cmd.mode = 1;
            custom.cmd.velocity[0] = 0;
            custom.cmd.velocity[1] = 0; 
            custom.cmd.yawSpeed = 0;
	    count++;
	    if(count < 5){
        	custom.udp.SetSend(custom.cmd);
        	custom.udp.Send();
	    }
        }
	    ros::spinOnce();
        loop_rate.sleep();
    };

    return 0; 
}
