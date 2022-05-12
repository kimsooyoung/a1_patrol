/*****
 * Blog: https://blog.csdn.net/qq_29797957?spm=1010.2135.3001.5343
 * Desp: 
 * Author: Ian.
 * Date: 2021-08-25 
 * ***********************/
#include <slam_planner/app_message.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapMessage_node");
	SLAM_MapMessage::SlamMapSocket mapObject;
	
	std::thread mp(bind(&SLAM_MapMessage::SlamMapSocket::runRos, &mapObject));
	mapObject.run(9802);
	
	mp.join();
	return 0;
}
