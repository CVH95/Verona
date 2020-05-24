/**
 * @file  perception.h
 *
 * Carlos Viescas.
 *
 * @brief ROS wrapper for detection of color markers.
 * @brief Publishes image points corresponding to marker centers.
 */

#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <string>
#include <vector>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

namespace perception_servoing
{
Eigen::Affine3d createAffineFromVectors(cv::Vec3d tvec, cv::Vec3d rvec);
cv::Rect getLargestRect(std::vector<cv::Rect> vec);
std::vector<cv::Point2f> getCornersFromRect(cv::Rect rectangle);
double euclideanDistance(Eigen::Affine3d a, Eigen::Affine3d b);
bool cvPointComparison(const cv::Point2f& a, const cv::Point2f& b);
cv::Point2f computeRectCenter(cv::Rect rect);
cv::Mat drawMarkerAxis(cv::Mat frame, Eigen::Affine3d pose, cv::Mat cam_k, cv::Mat cam_d);

bool findMarkerFromDots(cv::Mat& frame, Eigen::Affine3d& pose, double marker_size, cv::Mat cam_k, cv::Mat cam_d, int& e,
                        std::string& m);
bool findMarkerFromShape(cv::Mat& frame, Eigen::Affine3d& pose, double marker_size, cv::Mat cam_k, cv::Mat cam_d,
                         int& e, std::string& m);
bool computeFinalValues(Eigen::Affine3d pose_rb, Eigen::Affine3d pose_g, int& e, std::string& m,
                        Eigen::Affine3d& final_pose);
}  // namespace perception_servoing

#endif  // PERCEPTION_H