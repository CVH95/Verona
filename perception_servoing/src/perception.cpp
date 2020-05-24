/**
 * @file   perception.cpp
 *
 * Carlos Viescas.
 *
 * @brief ROS wrapper for detection of color markers.
 * @brief Publishes image points corresponding to marker centers.
 */

#include "perception_servoing/perception.h"

namespace perception_servoing
{
Eigen::Affine3d createAffineFromVectors(cv::Vec3d tvec, cv::Vec3d rvec)
{
  // Create rotation matrix
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(rvec[2], Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(rvec[1], Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(rvec[0], Eigen::Vector3d::UnitX());

  // Create translation vector
  Eigen::Vector3d position(tvec[0], tvec[1], tvec[2]);

  // Generate Transform
  Eigen::Affine3d affine;
  affine = rotation.matrix();
  affine.translation() = position;

  return affine;
}

cv::Rect getLargestRect(std::vector<cv::Rect> vec)
{
  cv::Rect max = vec[0];
  for (int i = 0; i < vec.size(); i++)
  {
    if (vec[i].width >= max.width && vec[i].height >= max.height)
    {
      max = vec[i];
    }
  }
  return max;
}

std::vector<cv::Point2f> getCornersFromRect(cv::Rect rectangle)
{
  std::vector<cv::Point2f> corners;
  cv::Point2f top_left(rectangle.x, rectangle.y);
  cv::Point2f top_right(rectangle.x + rectangle.width, rectangle.y);
  cv::Point2f bottom_left(rectangle.x, rectangle.y + rectangle.height);
  cv::Point2f bottom_right(rectangle.x + rectangle.width, rectangle.y + rectangle.height);

  corners.push_back(top_left);
  corners.push_back(top_right);
  corners.push_back(bottom_left);
  corners.push_back(bottom_right);
  return corners;
}

double euclideanDistance(Eigen::Affine3d a, Eigen::Affine3d b)
{
  Eigen::Vector3d v = a.translation();
  Eigen::Vector3d w = b.translation();

  double difference =
      (v.x() - w.x()) * (v.x() - w.x()) + (v.y() - w.y()) * (v.y() - w.y()) + (v.z() - w.z()) * (v.z() - w.z());
  return sqrt(difference);
}

bool cvPointComparison(const cv::Point2f& a, const cv::Point2f& b)
{
  return (a.x * a.x + a.y * a.y < b.x * b.x + b.y * b.y);
}

cv::Point2f computeRectCenter(cv::Rect rect)
{
  cv::Point2f center;
  center.x = rect.x + rect.width / 2;
  center.y = rect.y + rect.height / 2;
  return center;
}

cv::Mat drawMarkerAxis(cv::Mat frame, Eigen::Affine3d pose, cv::Mat cam_k, cv::Mat cam_d)
{
  cv::Mat output = frame;

  cv::Vec3d rvec, tvec;

  tvec[0] = pose.translation().x();
  tvec[1] = pose.translation().y();
  tvec[2] = pose.translation().z();

  Eigen::Matrix<double, 3, 1> euler = pose.rotation().eulerAngles(2, 1, 0);

  rvec[2] = euler(0, 0);
  rvec[1] = euler(1, 0);
  rvec[0] = euler(2, 0);

  cv::aruco::drawAxis(output, cam_k, cam_d, rvec, tvec, 0.2);
  return output;
}

bool findMarkerFromDots(cv::Mat& frame, Eigen::Affine3d& pose, double marker_size, cv::Mat cam_k, cv::Mat cam_d, int& e,
                        std::string& m)
{
  // Get camera params:
  double fx, fy;
  cv::Point2f image_center;
  fx = cam_k.at<double>(0, 0);
  fy = cam_k.at<double>(1, 1);
  image_center.x = cam_k.at<double>(0, 2);
  image_center.y = cam_k.at<double>(1, 2);

  // Initial blurr
  cv::GaussianBlur(frame, frame, cv::Size(3, 3), 2, 2);

  // Convert frame to HSV space
  cv::Mat hsv;
  cvtColor(frame, hsv, CV_BGR2HSV);

  // Red filter in HSV space.
  cv::Mat hue1, hue2;
  cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), hue1);
  cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), hue2);

  cv::Mat red_dot;
  cv::addWeighted(hue1, 1.0, hue2, 1.0, 0.0, red_dot);

  int m_size = 4;
  cv::Mat element =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(m_size + 1, m_size + 1), cv::Point(m_size, m_size));
  morphologyEx(red_dot, red_dot, cv::MORPH_OPEN, element, cv::Point(-1, -1), 6);

  // Blue filter in HSV
  cv::Mat blue_dot;
  cv::inRange(hsv, cv::Scalar(100, 50, 50), cv::Scalar(150, 255, 255), blue_dot);

  m_size = 2;
  element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(m_size + 1, m_size + 1), cv::Point(m_size, m_size));
  morphologyEx(blue_dot, blue_dot, cv::MORPH_OPEN, element, cv::Point(-1, -1), 6);

  // Combine both matrices
  cv::Mat red_n_blue;
  cv::addWeighted(red_dot, 1.0, blue_dot, 1.0, 0.0, red_n_blue);

  // Circle estimation
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(red_n_blue, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  std::vector<std::vector<cv::Point>> contours_poly(contours.size());
  std::vector<cv::Rect> bound_rect(contours.size());
  std::vector<cv::Point2f> center(contours.size());
  std::vector<float> radius(contours.size());

  for (int i = 0; i < contours.size(); i++)
  {
    cv::approxPolyDP(contours[i], contours_poly[i], 5, true);
    bound_rect[i] = cv::boundingRect(contours_poly[i]);
    cv::minEnclosingCircle(contours_poly[i], center[i], radius[i]);
  }

  if (center.size() == 0)
  {
    e = 404;
    m = "No markers found";
    return false;
  }

  if (center.size() > 4)
  {
    e = 405;
    m = "Too many centers found";
    return false;
  }

  // Sort vectors according to distance to image origin
  std::sort(center.begin(), center.end(), cvPointComparison);

  // Create cv::Rect from centers
  cv::Point2f top_left = center[0];
  cv::Point2f bottom_right = center[3];
  int r_w = bottom_right.x - top_left.x;
  int r_h = bottom_right.y - top_left.y;
  cv::Rect mini_marker(top_left.x, top_left.y, r_w, r_h);
  cv::Point2f keypoint = computeRectCenter(mini_marker);

  // Draw detected circles
  cv::Mat box = cv::Mat::zeros(red_n_blue.size(), CV_8UC3);
  for (int i = 0; i < contours.size(); i++)
  {
    cv::drawContours(box, contours, -1, cv::Scalar(0, 0, 255), 1);
    cv::circle(box, center[i], radius[i], cv::Scalar(0, 0, 255), -1, 8, 0);
    cv::circle(box, center[i], 6, cv::Scalar(0, 255, 255), -1, 8, 0);
  }
  cv::rectangle(box, mini_marker, cv::Scalar(0, 255, 255), 3, 8, 0);

  std::vector<std::vector<cv::Point2f>> marker_vec;
  marker_vec.push_back(center);

  // Estimate pose
  cv::Vec3d rvec(0, 0, 0);
  cv::Vec3d tvec;
  double depth_x = (double)(marker_size * 655.302) / mini_marker.width;
  double depth_y = (double)(marker_size * 655.302) / mini_marker.height;
  double depth = (depth_x + depth_y) / 2;

  tvec[2] = depth;                                         // Get Z
  tvec[0] = (keypoint.x - image_center.x) * (depth / fx);  // Get X
  tvec[1] = (keypoint.y - image_center.y) * (depth / fy);  // Get Y

  e = 0;
  m = "Success";
  pose = createAffineFromVectors(tvec, rvec);
  frame = box;

  std::cout << "Detected from color at: " << pose.translation().x() << ", " << pose.translation().y() << ", "
            << pose.translation().z() << std::endl;
  return true;
}

bool findMarkerFromShape(cv::Mat& frame, Eigen::Affine3d& pose, double marker_size, cv::Mat cam_k, cv::Mat cam_d,
                         int& e, std::string& m)
{
  // Get camera params:
  double fx, fy;
  cv::Point2f image_center;
  fx = cam_k.at<double>(0, 0);
  fy = cam_k.at<double>(1, 1);
  image_center.x = cam_k.at<double>(0, 2);
  image_center.y = cam_k.at<double>(1, 2);

  // Initial blurr
  cv::GaussianBlur(frame, frame, cv::Size(3, 3), 2, 2);

  // Convert frame to HSV space
  cv::Mat hsv;
  cvtColor(frame, hsv, CV_BGR2HSV);

  // Color segmentation in HSV space
  cv::Mat green;
  cv::inRange(hsv, cv::Scalar(40, 100, 100), cv::Scalar(80, 255, 255), green);

  int m_size = 2;
  cv::Mat element =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(m_size + 1, m_size + 1), cv::Point(m_size, m_size));
  morphologyEx(green, green, cv::MORPH_OPEN, element, cv::Point(-1, -1), 6);

  // Find marker shape
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(green, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  std::vector<std::vector<cv::Point>> contours_poly(contours.size());
  std::vector<cv::Rect> bound_rect(contours.size());

  for (int i = 0; i < contours.size(); i++)
  {
    cv::approxPolyDP(contours[i], contours_poly[i], 5, true);
    bound_rect[i] = cv::boundingRect(contours_poly[i]);
  }

  if (bound_rect.size() == 0)
  {
    e = 404;
    m = "No markers found";
    return false;
  }

  // Get largest bounding box
  cv::Rect marker_rect = getLargestRect(bound_rect);
  cv::Point2f keypoint = computeRectCenter(marker_rect);
  std::vector<cv::Point2f> corners = getCornersFromRect(marker_rect);

  cv::Mat box = cv::Mat::zeros(green.size(), CV_8UC3);
  cv::rectangle(box, marker_rect, cv::Scalar(0, 255, 255), 3, 8, 0);
  for (int i = 0; i < corners.size(); i++)
  {
    cv::circle(box, corners[i], 6, cv::Scalar(0, 0, 255), -1, 8, 0);
  }

  // Estimate pose
  cv::Vec3d rvec(0, 0, 0);
  cv::Vec3d tvec;
  double depth_x = (double)(marker_size * 655.302) / marker_rect.width;
  double depth_y = (double)(marker_size * 655.302) / marker_rect.height;
  double depth = (depth_x + depth_y) / 2;

  tvec[2] = depth;                                         // Get Z
  tvec[0] = (keypoint.x - image_center.x) * (depth / fx);  // Get X
  tvec[1] = (keypoint.y - image_center.y) * (depth / fy);  // Get Y

  e = 0;
  m = "Success";
  pose = createAffineFromVectors(tvec, rvec);
  frame = box;

  std::cout << "Detected from shape at: " << pose.translation().x() << ", " << pose.translation().y() << ", "
            << pose.translation().z() << std::endl;
  return true;
}

bool computeFinalValues(Eigen::Affine3d pose_rb, Eigen::Affine3d pose_g, int& e, std::string& m,
                        Eigen::Affine3d& final_pose)
{
  double dist = euclideanDistance(pose_rb, pose_g);
  if (dist >= 1.0)
  {
    std::cerr << "Detection too far appart!! distance = " << dist << " Skipping frame!!" << std::endl;
    e = 300;
    m = "Conflict between detections. Distance between them = " + std::to_string(dist) + ". Skipping frame!";
    return false;
  }

  final_pose = pose_g.matrix();
  final_pose.translation().x() = (pose_rb.translation().x() + pose_g.translation().x()) / 2;
  final_pose.translation().y() = (pose_rb.translation().y() + pose_g.translation().y()) / 2;
  final_pose.translation().z() = (pose_rb.translation().z() + pose_g.translation().z()) / 2;

  e = 0;
  m = "Success";
  return true;
}
}  // namespace perception_servoing