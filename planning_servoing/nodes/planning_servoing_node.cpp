/**
 * @file  planning_servoing.h
 *
 * Carlos.
 *
 * @brief ROS planner for servoing.
 * @brief Uses moveit.
 */

#include "planning_servoing/planning_servoing.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_servoing_node");
  ros::NodeHandle nh;

  planning_servoing::Planner planner(nh);

  planner.initSubscribers();

  ros::spin();
  return 0;
}