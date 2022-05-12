/*****
 * Blog: https://blog.csdn.net/qq_29797957?spm=1010.2135.3001.5343
 * Desp: 
 * Author: Ian.
 * Date: 2021-08-25 
 * ***********************/

#include <slam_planner/app_message.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mqttControler_node");
// 	SLAM_MqttControler::MqttController mqttObject;
	std::string mqttServerAddress = "tcp://192.168.123.161:1883";
	SLAM_MqttControler::MqttController mqttObject(mqttServerAddress);
	ros::spin();
	return 0;
}
