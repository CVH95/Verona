/**
 * @file   vision_server_ros.h
 *
 * @author Carlos
 *
 * @brief ROS template for vision service interface.
 * @brief Uses OpenCV.
 */

#ifndef VISION_SERVER_ROS
#define VISION_SERVER_ROS

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "vision_lib/vision_utils.h"
#include "vision_lib_msgs/BasicService.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"

#include "perception_servoing/perception.h"

namespace vision_service_ros
{
class VisionServerRos
{
private:
  ros::NodeHandle nh_;

  // Service
  ros::ServiceServer server_;

  // Pubs & Subs
  ros::Publisher color_pub_, image_pub_;

  // Params
  std::string service_id_;
  std::string img_topic_;
  std::string calibration_;
  bool publish_images_;
  double marker_size_, center_distance_;

  // Camera data
  int img_w_, img_h_;
  double fx_, fy_;
  cv::Point2f image_center_;
  cv::Mat intrinsics_;
  cv::Mat distorsion_;
  std::string camera_frame_;

  // Globals
  std::string service_path_;
  cv::Mat frame_;
  int error_code_;
  std::string error_message_;

  // Callbacks
  sensor_msgs::Image::ConstPtr shared_msg_;
  bool imageCallback();
  bool serviceHandler(vision_lib_msgs::BasicService::Request& req, vision_lib_msgs::BasicService::Response& resp);
  void sendTf(geometry_msgs::TransformStamped ts);

public:
  VisionServerRos(ros::NodeHandle node_handle);
  ~VisionServerRos();

  bool readParams();
  bool readCalibrationFile();
  bool activateSubscriber();

  void initServer();
  void initPublishers();
};
}  // namespace vision_service_ros

#endif  // VISION_SERVER_ROS