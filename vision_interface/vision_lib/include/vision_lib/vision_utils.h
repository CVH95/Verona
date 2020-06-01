/**
 * @file   client_utils.h
 *
 * @author Carlos
 *
 * @brief ROS template for vision service interface.
 * @brief Uses OpenCV.
 */

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "vision_lib_msgs/BasicService.h"

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

namespace vision_service_ros
{
// Image format conversion
cv::Mat convertSensorMsgsToMat(sensor_msgs::Image msg);
sensor_msgs::Image convertMatToSensorMsgs(cv::Mat image);

// Tf broadcasting
void sendSingleTf(Eigen::Affine3d pose, std::string camera_frame, std::string child_frame);
}  // namespace vision_service_ros