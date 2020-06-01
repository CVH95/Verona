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
  ros::init(argc, argv, "planner_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  planning_servoing::Planner planner(nh);

  planner.initSubscribers();

  ros::waitForShutdown();
  return 0;
}