/**
 * @file   vision_server_ros.cpp
 *
 * @author Carlos
 *
 * @brief ROS template for vision service interface.
 * @brief Uses OpenCV.
 */

#include "vision_server_ros/vision_server_ros.h"

namespace vision_service_ros
{
VisionServerRos::VisionServerRos(ros::NodeHandle node_handle) : nh_(node_handle)
{
  if (!readParams())
  {
    ROS_ERROR("Could not retrieve data from parameter server!");
    exit(1);
  }

  if (!readCalibrationFile())
  {
    ROS_ERROR("Could not read calibration file!");
    exit(1);
  }

  service_path_ = service_id_ + "/marker";

  ROS_INFO("--- VISION SERVER ---");
  ROS_INFO(" * Service: %s", service_id_.c_str());
  ROS_INFO(" * Camera calibration file: %s", calibration_.c_str());
  ROS_INFO(" * Images streaming on: %s", img_topic_.c_str());
  ROS_INFO(" * Marker size: %f", marker_size_);
  ROS_INFO(" * Distance between color dot centers: %f", center_distance_);
  ROS_INFO(" * Publish intermediate images: %s", publish_images_ ? "ENABLED" : "DISABLED");
  ROS_INFO("Camera data:");
  ROS_INFO(" * Camera frame: %s", camera_frame_.c_str());
  ROS_INFO(" * Width: %i", img_w_);
  ROS_INFO(" * Height: %i", img_h_);
  ROS_INFO(" * Fx: %f", fx_);
  ROS_INFO(" * Fy: %f", fy_);
  ROS_INFO(" * Image center: %f, %f", image_center_.x, image_center_.y);
  ROS_INFO("\n---------\n");

  initPublishers();
}

VisionServerRos::~VisionServerRos()
{
  ROS_WARN("Vision Server Killed!");
}

bool VisionServerRos::readParams()
{
  bool success = true;
  success = success && ros::param::get("service_id", service_id_);
  success = success && ros::param::get("img_topic", img_topic_);
  success = success && ros::param::get("calibration", calibration_);
  success = success && ros::param::get("publish_images", publish_images_);
  success = success && ros::param::get("marker_size", marker_size_);
  success = success && ros::param::get("center_distance", center_distance_);
  return success;
}

bool VisionServerRos::readCalibrationFile()
{
  // Obtain extrinsics
  cv::FileStorage fs(calibration_, cv::FileStorage::READ);

  fs["image_width"] >> img_w_;
  fs["image_height"] >> img_h_;
  fs["camera_frame"] >> camera_frame_;
  fs["camera_matrix"] >> intrinsics_;
  fs["distortion_coefficients"] >> distorsion_;
  fs.release();

  fx_ = intrinsics_.at<double>(0, 0);
  fy_ = intrinsics_.at<double>(1, 1);
  image_center_.x = intrinsics_.at<double>(0, 2);
  image_center_.y = intrinsics_.at<double>(1, 2);

  if (intrinsics_.empty() || distorsion_.empty())
  {
    return false;
  }
  return true;
}

bool VisionServerRos::activateSubscriber()
{
  shared_msg_ = ros::topic::waitForMessage<sensor_msgs::Image>(img_topic_, nh_);
  bool rec = imageCallback();
  return rec;
}

void VisionServerRos::initPublishers()
{
  if (publish_images_)
  {
    std::string color_topic_ = service_id_ + "/color_detection_result";
    std::string result_topic_ = service_id_ + "/pose_estimation_result";

    color_pub_ = nh_.advertise<sensor_msgs::Image>(color_topic_, 1);
    image_pub_ = nh_.advertise<sensor_msgs::Image>(result_topic_, 1);
  }
}

void VisionServerRos::initServer()
{
  server_ = nh_.advertiseService(service_path_, &VisionServerRos::serviceHandler, this);
}

bool VisionServerRos::imageCallback()
{
  cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(*shared_msg_, sensor_msgs::image_encodings::BGR8);
  frame_ = img_ptr->image;
  if (frame_.empty())
  {
    error_code_ = 200;
    error_message_ = "Received empty image";
    return false;
  }
  return true;
}

bool VisionServerRos::serviceHandler(vision_lib_msgs::BasicService::Request& req,
                                     vision_lib_msgs::BasicService::Response& resp)
{
  if (req.call_id != service_id_)
  {
    ROS_WARN("Unknown '%s' request received!!", req.call_id.c_str());
    resp.error_code = 100;
    resp.error_message = "Unknown service";
    return false;
  }
  ROS_INFO("Received request for '%s'", req.call_id.c_str());

  bool success = activateSubscriber();

  if (!success)
  {
    resp.error_code = error_code_;
    resp.error_message = error_message_;
    return false;
  }

  sensor_msgs::ImagePtr raw_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_).toImageMsg();

  /* CALL TO DETECTION RUTINE */
  bool success_color = true;
  bool success_shape = true;
  cv::Mat frame_color = frame_;
  cv::Mat frame_shape = frame_;
  cv::Mat draw_pose = frame_;
  Eigen::Affine3d pose_color, pose_shape, final_pose;

  success_color = perception_servoing::findMarkerFromDots(frame_color, pose_color, center_distance_, intrinsics_,
                                                          distorsion_, error_code_, error_message_);
  success_shape = perception_servoing::findMarkerFromShape(frame_shape, pose_shape, marker_size_, intrinsics_,
                                                           distorsion_, error_code_, error_message_);

  // No detections
  if (!success_color && !success_shape)
  {
    resp.capture = *raw_img;
    resp.error_code = error_code_;
    resp.error_message = error_message_;

    if (publish_images_)
    {
      color_pub_.publish(raw_img);
      image_pub_.publish(raw_img);
    }

    return false;
  }

  // Only shape detection
  if (!success_color && success_shape)
  {
    cv::Mat final_img = perception_servoing::drawMarkerAxis(draw_pose, pose_shape, intrinsics_, distorsion_);
    sensor_msgs::ImagePtr shape_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_shape).toImageMsg();
    sensor_msgs::ImagePtr pose_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final_img).toImageMsg();

    geometry_msgs::TransformStamped marker_pose;
    tf::transformEigenToMsg(pose_shape, marker_pose.transform);

    resp.capture = *pose_img;
    resp.marker_pose = marker_pose.transform;
    resp.error_code = 1;
    resp.error_message = "Detected marker only from shape";

    marker_pose.header.frame_id = camera_frame_;
    marker_pose.child_frame_id = "marker";
    marker_pose.header.stamp = ros::Time::now();
    sendTf(marker_pose);

    if (publish_images_)
    {
      color_pub_.publish(raw_img);
      image_pub_.publish(pose_img);
    }

    frame_.release();
    return false;
  }

  // Only color detection
  if (success_color && !success_shape)
  {
    cv::Mat final_img = perception_servoing::drawMarkerAxis(draw_pose, pose_color, intrinsics_, distorsion_);
    sensor_msgs::ImagePtr color_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_color).toImageMsg();
    sensor_msgs::ImagePtr pose_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final_img).toImageMsg();

    geometry_msgs::TransformStamped marker_pose;
    tf::transformEigenToMsg(pose_color, marker_pose.transform);

    resp.capture = *pose_img;
    resp.marker_pose = marker_pose.transform;
    resp.error_code = 2;
    resp.error_message = "Detected marker only from color dots";

    marker_pose.header.frame_id = camera_frame_;
    marker_pose.child_frame_id = "marker";
    marker_pose.header.stamp = ros::Time::now();
    sendTf(marker_pose);

    if (publish_images_)
    {
      color_pub_.publish(color_img);
      image_pub_.publish(pose_img);
    }

    return false;
  }

  cv::Mat pb_color_img;
  cv::addWeighted(frame_shape, 1.0, frame_color, 1.0, 0.0, pb_color_img);
  sensor_msgs::ImagePtr color_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pb_color_img).toImageMsg();

  // both detections
  success = perception_servoing::computeFinalValues(pose_color, pose_shape, error_code_, error_message_, final_pose);
  if (!success)
  {
    resp.capture = *raw_img;
    resp.error_code = error_code_;
    resp.error_message = error_message_;

    if (publish_images_)
    {
      color_pub_.publish(color_img);
      image_pub_.publish(raw_img);
    }

    return false;
  }

  // Compose final message
  cv::Mat final_img = perception_servoing::drawMarkerAxis(draw_pose, final_pose, intrinsics_, distorsion_);
  sensor_msgs::ImagePtr pose_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final_img).toImageMsg();

  geometry_msgs::TransformStamped marker_pose;
  tf::transformEigenToMsg(final_pose, marker_pose.transform);

  resp.capture = *pose_img;
  resp.marker_pose = marker_pose.transform;
  resp.error_code = 2;
  resp.error_message = "Detected marker only from color dots";

  marker_pose.header.frame_id = camera_frame_;
  marker_pose.child_frame_id = "marker";
  marker_pose.header.stamp = ros::Time::now();
  sendTf(marker_pose);

  if (publish_images_)
  {
    color_pub_.publish(color_img);
    image_pub_.publish(pose_img);
  }

  /* CALL TO DETECTION RUTINE */

  return true;
}

void VisionServerRos::sendTf(geometry_msgs::TransformStamped ts)
{
  static tf2_ros::StaticTransformBroadcaster br;
  br.sendTransform(ts);
}
}  // namespace vision_service_ros