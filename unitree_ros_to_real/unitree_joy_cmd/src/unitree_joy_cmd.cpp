/*********************************************************************
 * Software License
 *  Copyright (c) 2022, WeGo Robotices Co., Ltd. && Road Balance Co. All Rights Reserved.
 *********************************************************************/


#include "unitree_joy_cmd/UnitreeJoyCmd.hpp"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "unitree_joy_cmd");

  ros::NodeHandle nh;
  UnitreeJoyCmd joy_to_cmd_node{&nh};

  ros::spin();
}