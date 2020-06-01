/**
 * @file   vision_server_ros.cpp
 *
 * @author Carlos
 *
 * @brief ROS template for vision service interface.
 * @brief Uses OpenCV.
 */

#include "vision_server_ros/vision_server_ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_server_ros_node");
  ros::NodeHandle nh;

  vision_service_ros::VisionServerRos server(nh);
  server.initServer();

  ros::spin();
  return 0;
}