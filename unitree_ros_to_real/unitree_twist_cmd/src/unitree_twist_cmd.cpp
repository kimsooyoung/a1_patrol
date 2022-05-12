/*********************************************************************
 * Software License
 *  Copyright (c) 2022, WeGo Robotices Co., Ltd. && Road Balance Co. All Rights Reserved.
 *********************************************************************/

#include "unitree_twist_cmd/UnitreeTwistCmd.hpp"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "unitree_twist_cmd");

  ros::NodeHandle nh;
  UnitreeTwistCmd unitree_twist_cmd(&nh);
  ros::spin();

  ros::shutdown();

  return 0;
}