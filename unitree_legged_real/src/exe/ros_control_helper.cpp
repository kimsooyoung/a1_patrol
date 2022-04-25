/*********************************************************************
 * Software License
 *  Copyright (c) 2022, WeGo Robotices Co., Ltd. && Road Balance Co. All Rights Reserved.
 *********************************************************************/

#include "HighLevelHelper.hpp"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "dog_control_node");
  ros::NodeHandle nh;

  UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);

  HighLevelHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM> main_helper(&nh, roslcm);
  ros::spin();

  return 0;
}
