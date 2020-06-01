/**
 * @file  planning_servoing.h
 *
 * Carlos.
 *
 * @brief ROS planner for servoing.
 * @brief Uses moveit.
 */

#include "planning_servoing/planning_servoing.h"

namespace planning_servoing
{
Planner::Planner(ros::NodeHandle node_handle) : nh_(node_handle)
{
  if (!readParams())
  {
    ROS_ERROR("Could not retrieve from parameter server!");
    exit(1);
  }

  ROS_INFO("--- ANYTIME PLANNER ---");
  ROS_INFO(" * Subscribed to: %s", marker_motion_topic_.c_str());
  ROS_INFO(" * Moveit file: %s", moveit_file_.c_str());
  ROS_INFO(" * Camera used: %s", camera_frame_.c_str());
  ROS_INFO(" * Moveit file: %s", service_id_.c_str());
  ROS_INFO(" * Camera used: %s", service_path_.c_str());
  ROS_INFO("MoveGroup info:");
  ROS_INFO(" * Planning group: %s", planning_group_.c_str());

  // Read moveit configuration file
  if (!setMoveitConfiguration())
  {
    ROS_ERROR("Could not read moveit configuration!");
    exit(1);
  }

  // Start moveit
  mgp_.reset(new moveit::planning_interface::MoveGroupInterface(planning_group_));
  planning_scene_interface_.reset(new moveit::planning_interface::PlanningSceneInterface);

  // Set robot model
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model_ = robot_model_loader.getModel();
  joint_model_group_ = mgp_->getCurrentState()->getJointModelGroup(planning_group_);

  // Configure planning
  mgp_->setPoseReferenceFrame(planning_frame_);
  mgp_->setEndEffectorLink(end_effector_frame_);
  mgp_->setPlannerId(planner_id_);

  // Allow RVIZ visualization
  moveit_visual_tools::MoveItVisualTools visual_tools(mgp_->getPlanningFrame().c_str());
  visual_tools.loadRemoteControl();
  visual_tools.trigger();

  ros::Duration(2.0).sleep();  // Just to let everything settle

  if (!moveRobot(home_position_))
  {
    ROS_ERROR("Could not set robot in home position!");
    exit(1);
  }

  // Tf listener
  tf_buffer_.reset(new tf2_ros::Buffer());
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_, root_nh_));

  ros::Duration(2.0).sleep();  // Just to let everything settle

  geometry_msgs::TransformStamped ts;
  try
  {
    ts = tf_buffer_->lookupTransform(end_effector_frame_, camera_frame_, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("Could not get transform from end effector to camera!");
    exit(1);
  }
  tf::transformMsgToEigen(ts.transform, ee_to_camera_);

  initClient();
  ROS_INFO("All set and ready!");
}

Planner::~Planner()
{
  delete joint_model_group_;
  ROS_WARN("Planner node was killed!");
}

bool Planner::readParams()
{
  bool success = true;
  success = success && ros::param::get("planning_group", planning_group_);
  success = success && ros::param::get("marker_motion_topic", marker_motion_topic_);
  success = success && ros::param::get("camera_frame", camera_frame_);
  success = success && ros::param::get("moveit_file", moveit_file_);
  success = success && ros::param::get("service_id", service_id_);
  success = success && ros::param::get("service_path", service_path_);
  return success;
}

bool Planner::setMoveitConfiguration()
{
  YAML::Node map = YAML::LoadFile(moveit_file_.c_str());
  std::vector<double> pick, place, approach;

  YAML::Node entry = map["planning_frame"];
  planning_frame_ = entry[0]["value"].as<std::string>();
  ROS_INFO(" * Planning reference frame: %s", planning_frame_.c_str());

  entry = map["end_effector_frame"];
  end_effector_frame_ = entry[0]["value"].as<std::string>();
  ROS_INFO(" * End Effector frame: %s", end_effector_frame_.c_str());

  entry = map["planner_id"];
  planner_id_ = entry[0]["value"].as<std::string>();
  ROS_INFO(" * Selected planner: %s", planner_id_.c_str());

  entry = map["home_position"];
  home_position_ = entry[0]["value"].as<std::vector<double>>();
  ROS_INFO(" * Home position: [%f, %f, %f, %f, %f, %f]", home_position_[0], home_position_[1], home_position_[2],
           home_position_[3], home_position_[4], home_position_[5]);

  if (home_position_.empty())
  {
    return false;
  }

  ROS_INFO("Moveit configuration parsed successfully!");
  return true;
}

void Planner::initSubscribers()
{
  marker_motion_sub_ = nh_.subscribe(marker_motion_topic_, 1, &Planner::markerCallback, this);
}

void Planner::initClient()
{
  client_ = nh_.serviceClient<vision_lib_msgs::BasicService>(service_path_);
}

bool Planner::moveRobot(std::vector<double> joint_positions)
{
  mgp_->setStartState(*mgp_->getCurrentState());
  mgp_->setJointValueTarget(joint_positions);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (mgp_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_DEBUG("Planning status: %s", success ? "SUCCESS" : "FAILED");
  if (success)
  {
    success = (mgp_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_DEBUG("Moved camera to new pose %s", success ? "SUCCESS" : "FAILED");
  }
  return success;
}

bool Planner::moveRobotToPose(geometry_msgs::Pose pose)
{
  mgp_->setPoseTarget(pose, end_effector_frame_);
  bool success = (mgp_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_DEBUG("Moved camera to new pose %s", success ? "SUCCESS" : "FAILED");
  return success;
}

bool Planner::solveIK(geometry_msgs::Pose pose, std::vector<double>& joint_positions)
{
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model_));
  kinematic_state->setToDefaultValues();

  bool success = kinematic_state->setFromIK(joint_model_group_, pose, 10, 0.1);
  if (!success)
  {
    return false;
  }

  kinematic_state->copyJointGroupPositions(joint_model_group_, joint_positions);
  return true;
}

void Planner::markerCallback(const std_msgs::Bool& msg)
{
  if (msg.data == false)
  {
    return;
  }

  serviceCallHandler();
}

void Planner::serviceCallHandler()
{
  vision_lib_msgs::BasicService basic_srv;
  basic_srv.request.call_id = service_id_;
  if (!client_.call(basic_srv))
  {
    ROS_ERROR("Could not recieve responce from Server.");
    return;
  }

  int code = basic_srv.response.error_code;
  std::string mess = basic_srv.response.error_message;
  if (code > 10)
  {
    ROS_WARN("Received error %i with message: %s", code, mess.c_str());
    return;
  }

  ROS_INFO("Received error %i with message: %s", code, mess.c_str());

  geometry_msgs::TransformStamped current, marker, camera;
  marker.transform = basic_srv.response.marker_pose;
  try
  {
    current = tf_buffer_->lookupTransform(planning_frame_, end_effector_frame_, ros::Time(0));
    // marker = tf_buffer_->lookupTransform(planning_frame_, "marker", ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("Could not get current robot pose!");
    return;
  }

  // Calculate transform of marker in world reference
  Eigen::Affine3d current_h, marker_h;
  tf::transformMsgToEigen(current.transform, current_h);
  tf::transformMsgToEigen(marker.transform, marker_h);
  Eigen::Affine3d world_to_marker = current_h * ee_to_camera_ * marker_h;  // T_0_3 = T_0_1 * T_1_2 * T_2_3

  // Update new robot pose
  current_h.translation().y() = world_to_marker.translation().y();
  current_h.translation().z() = world_to_marker.translation().z();

  geometry_msgs::Pose goal;
  tf::poseEigenToMsg(current_h, goal);

  bool success;

  // Manual IK solving
  /*std::vector<double> new_joint_positions;
  success = solveIK(goal, new_joint_positions);
  if(!success)
  {
    ROS_ERROR("Could not find solution to IK!");
    return;
  }
  success = moveRobot(new_joint_positions);*/

  success = moveRobotToPose(goal);
  if (!success)
  {
    ROS_ERROR("Could not move robot to new state!");
  }
  ROS_INFO("New pose: %f, %f, %f", current_h.translation().x(), current_h.translation().y(),
           current_h.translation().z());
}

}  // namespace planning_servoing