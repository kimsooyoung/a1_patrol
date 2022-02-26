#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <std_msgs/UInt8.h>
#include <tf2/LinearMath/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// 本程序用于通过 ros 发布巡逻坐标点，使用了 move_base 的 actionlib 库，相比单纯的 topic 发布巡逻点的优点是可以知道任务执行的结果

// 创建 任务数据类型
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
	// 初始化 ros
	ros::init(argc, argv, "simple_navigation_goals");
	ros::NodeHandle n;

	// 创建要执行任务的客户端
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	// 等待 任务服务端 启动
	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
	ROS_INFO("Waiting for the move_base action server to come up");
	}

	// 创建一系列所需要的数据
	move_base_msgs::MoveBaseGoal goal;
	std::string patrol_points_file_path;
	float point_x = 0;
	float point_y = 0;
	float yaw = 0;
	tf2::Quaternion dog_pose;
	float stay_time = 0;
	std_msgs::UInt8 stand;
	std_msgs::UInt8 walk;
	stand.data = 1;
	walk.data = 2;

	// 创建 topic 发布器
	ros::Publisher dog_mode_pub = n.advertise<std_msgs::UInt8>("/dog_mode", 10);

	// 获取巡逻点所存放文件的路径
	ros::param::get("send_patrol_points/patrol_points_file", patrol_points_file_path);
	// 读取巡逻点存放文件
	std::ifstream patrol_points_file(patrol_points_file_path.c_str());
	while (true) {
		// 从文件中读取巡逻点数据
		patrol_points_file >> point_x;
		patrol_points_file >> point_y;
		patrol_points_file >> yaw;
		patrol_points_file >> stay_time;

		// 当数据读取完之后，结束循环
		if (patrol_points_file.eof()) {
			break;
		}

		// 设置狗的位姿，将用 欧拉角 表示的位姿 转换为用 四元数 表示的位姿
		dog_pose.setRPY(0, 0, yaw);
		std::cout << "*****************" << point_x 
					<< " "
					<< point_y
					<< " "
					<< yaw
					<< " "
					<< stay_time << std::endl;

		// 构建巡逻点 topic 数据
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = point_x;
		goal.target_pose.pose.position.y = point_y;
		goal.target_pose.pose.orientation.x = dog_pose.x();
		goal.target_pose.pose.orientation.y = dog_pose.y();
		goal.target_pose.pose.orientation.z = dog_pose.z();
		goal.target_pose.pose.orientation.w = dog_pose.w();

		// 在命令行中输出信息
		ROS_INFO("Sending goal");
		// 发布巡逻任务点
		ac.sendGoal(goal);
		// 等待结果，ros::Duration(0) 表示无限期，可以设置其他值，单位为 秒
		ac.waitForResult(ros::Duration(0));

		// 检查 任务的执行结果
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("Hooray, the base moved to the POINT!");
			// 如果任务成功之后，让狗站住
			dog_mode_pub.publish(stand);
			// 停留一些时间，时长为：stay_time 
			ros::Duration(stay_time).sleep();
			// 让狗继续开始走路
			dog_mode_pub.publish(walk);
		} else {
			ROS_INFO("The base failed to move to the POINT for some reason");
		}
	}
	// 循环结束之后让狗站住不动
	dog_mode_pub.publish(stand);
	ros::spin();

	return 0;
}
