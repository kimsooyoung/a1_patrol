/*****
 * Blog: https://blog.csdn.net/qq_29797957?spm=1010.2135.3001.5343
 * Desp: 
 * Author: Ian.
 * Date: 2021-08-25 
 * ***********************/

#ifndef __APP_MESSAGE_HPP__
#define __APP_MESSAGE_HPP__

#include <set>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <slam_planner/lzjb.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include <move_base_msgs/MoveBaseAction.h>  

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <websocketpp/common/thread.hpp>

#include <mqtt/async_client.h>


/**
* @namespace SLAM_MapMessage
* @brief Send map message to app.
*/
namespace SLAM_MapMessage{

typedef websocketpp::server<websocketpp::config::asio> server;
using websocketpp::connection_hdl;
using websocketpp::lib::bind;
using websocketpp::lib::mutex;
using websocketpp::lib::thread;
using websocketpp::lib::unique_lock;
using websocketpp::lib::placeholders::_1;

/**
 * @class SlamMapSocket
 * @brief map handle class.
 */
class SlamMapSocket{
	public:
		SlamMapSocket();

		void on_open(connection_hdl hdl);
		
		void on_close(connection_hdl hdl);
		
		void run(uint16_t port);
		
		void runRos();
		
		/**
		 * @brief  receive nav_msgs::OccupancyGrid message.
		 * @param [in] "/map"
		 */
		void mapCallback(const nav_msgs::OccupancyGridConstPtr &map);
		
		void costmapCallback(const nav_msgs::OccupancyGridConstPtr &costmap);
		
		void bufferMap(const nav_msgs::OccupancyGridConstPtr &map, bool isCostmap);
		
		void sendMap(const nav_msgs::OccupancyGridConstPtr &map, bool isCostmap);
		
		bool checkMap();
		
	private:
		ros::NodeHandle nh;
		ros::Subscriber subMap;
		ros::Subscriber subCostmap;
		typedef std::set<connection_hdl, std::owner_less<connection_hdl>> con_list;
		server m_server;
		con_list m_connections;
		mutex m_connection_lock;
		nav_msgs::MapMetaData m_mapMeta;
		nav_msgs::MapMetaData m_costmapMeta;
		bool isSent;
		uint8_t *mapBuffer;
		uint8_t *sendBuffer;
		size_t sendBufferSize;
		int pingCount;	
	};
};

/**
* @namespace SLAM_MqttControler
* @brief 
*/
namespace SLAM_MqttControler{
	class MqttController{
	public:
		MqttController();

		MqttController(std::string ipAddr);
		
		void init(std::string ipAddr, std::string clientID);
		
		void mqttConnectedCallback();
		
		void pubTarget(geometry_msgs::Pose pose);
		
		void mqttMsgCallback(mqtt::const_message_ptr msg);
		
		void odomCallback(const nav_msgs::OdometryConstPtr &odom);
		
		void navFeedback(const move_base_msgs::MoveBaseActionResult &navResult);
		
	private:
		ros::NodeHandle nh;
		ros::Publisher pub;
		ros::Subscriber sub;
		ros::Subscriber subNav;
		mqtt::async_client *cli;
		geometry_msgs::Pose targets[20];
		geometry_msgs::Pose currentPose;
		size_t walkingProgress;
		size_t targetsCount;
		char targetIndex;
		bool isLooping;
		
		int navStatus[20];
	};
};


#endif