/**
 * @file   vision_utils.cpp
 *
 * @author Carlos
 *
 * @brief ROS template for vision service interface.
 * @brief Uses OpenCV.
 */

#include "vision_lib/vision_utils.h"

namespace vision_service_ros
{
cv::Mat convertSensorMsgsToMat(sensor_msgs::Image msg)
{
  cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat image = img_ptr->image;
  return image;
}

sensor_msgs::Image convertMatToSensorMsgs(cv::Mat image)
{
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  return *msg;
}

void sendSingleTf(Eigen::Affine3d pose, std::string camera_frame, std::string child_frame)
{
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.child_frame_id = child_frame;
  transform_stamped.header.frame_id = camera_frame;
  transform_stamped.header.stamp = ros::Time::now();
  tf::transformEigenToMsg(pose, transform_stamped.transform);

  static tf2_ros::StaticTransformBroadcaster br;
  br.sendTransform(transform_stamped);
}
}  // namespace vision_service_ros