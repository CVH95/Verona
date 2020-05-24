/**
 * @file  planning_servoing.h
 *
 * Carlos.
 *
 * @brief ROS planner for servoing.
 * @brief Uses moveit.
 */

#ifndef PLANNING_SERVOING
#define PLANNING_SERVOING

#include <string>
#include <vector>
#include <iostream>
#include <yaml-cpp/yaml.h>

#include <ros/package.h>
#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"
#include "vision_lib_msgs/BasicService.h"

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

namespace planning_servoing
{
class Planner
{
private:
  ros::NodeHandle nh_;

  // Pubs & Subs
  ros::Subscriber marker_motion_sub_;

  // Service
  ros::ServiceClient client_;

  // Tf
  ros::NodeHandle root_nh_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Params
  std::string planning_group_, marker_motion_topic_, camera_frame_, moveit_file_;
  std::string service_id_, service_path_;

  // Moveit
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> mgp_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  robot_model::RobotModelPtr robot_model_;
  std::string planning_frame_, end_effector_frame_, planner_id_;
  std::vector<double> home_position_;
  Eigen::Affine3d ee_to_camera_;

  // Callbacks
  void markerCallback(const std_msgs::Bool& msg);
  void serviceCallHandler();

  // Private methods
  bool moveRobot(std::vector<double> joint_positions);
  bool moveRobotToPose(geometry_msgs::Pose pose);

public:
  Planner(ros::NodeHandle node_handle);
  ~Planner();

  bool readParams();
  void setMoveitConfiguration();
  void initSubscribers();
  void initClient();
};
}  // namespace planning_servoing

#endif  // PLANNING_SERVOING