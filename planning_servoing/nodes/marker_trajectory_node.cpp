/**
 * @file   marker_trajectory.cpp
 *
 * Carlos Viescas.
 *
 * @brief Generates trajectory for marker.
 */

#include <math.h>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "std_msgs/Bool.h"
#include "moveit_msgs/CollisionObject.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "gazebo_msgs/LinkState.h"

double doubleRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

void sendTf(geometry_msgs::Pose pose)
{
  geometry_msgs::TransformStamped ts;

  ts.transform.translation.x = pose.position.x;
  ts.transform.translation.y = pose.position.y;
  ts.transform.translation.z = pose.position.z;
  ts.transform.rotation.x = pose.orientation.x;
  ts.transform.rotation.y = pose.orientation.y;
  ts.transform.rotation.z = pose.orientation.z;
  ts.transform.rotation.w = pose.orientation.w;

  ts.header.stamp = ros::Time::now();
  ts.header.frame_id = "world";
  ts.child_frame_id = "marker_link";

  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(ts);
}

bool triangle(geometry_msgs::Pose& pose, int& step)
{
  int limit = 8;
  if (step == limit)
  {
    step = 0;
    return false;
  }
  switch (step)
  {
    case 0:
      pose.position.x = 1.25;
      pose.position.y = 0.25;
      pose.position.z = 1.70;
      step++;
      break;
    case 1:
      pose.position.y = 0.35;
      pose.position.z = 1.70;
      step++;
      break;
    case 2:
      pose.position.y = 0.45;
      pose.position.z = 1.70;
      step++;
      break;
    case 3:
      pose.position.y = 0.35;
      pose.position.z = 1.80;
      step++;
      break;
    case 4:
      pose.position.y = 0.25;
      pose.position.z = 1.90;
      step++;
      break;
    case 5:
      pose.position.y = 0.15;
      pose.position.z = 1.80;
      step++;
      break;
    case 6:
      pose.position.y = 0.05;
      pose.position.z = 1.70;
      step++;
      break;
    case 7:
      pose.position.y = 0.15;
      pose.position.z = 1.70;
      step++;
      break;
    default:
      break;
  }
  return true;
}

bool square(geometry_msgs::Pose& pose, int& step)
{
  int limit = 20;
  if (step >= limit)
  {
    step = 0;
    return false;
  }
  switch (step)
  {
    case 0:
      pose.position.x = 1.25;
      pose.position.y = 0.25;
      pose.position.z = 1.70;
      step++;
      break;
    case 1:
      pose.position.y = 0.35;
      pose.position.z = 1.70;
      step++;
      break;
    case 2:
      pose.position.y = 0.45;
      pose.position.z = 1.70;
      step++;
      break;
    case 3:
      pose.position.y = 0.45;
      pose.position.z = 1.80;
      step++;
      break;
    case 4:
      pose.position.y = 0.45;
      pose.position.z = 1.90;
      step++;
      break;
    case 5:
      pose.position.y = 0.35;
      pose.position.z = 1.90;
      step++;
      break;
    case 6:
      pose.position.y = 0.25;
      pose.position.z = 1.90;
      step++;
      break;
    case 7:
      pose.position.y = 0.15;
      pose.position.z = 1.90;
      step++;
      break;
    case 8:
      pose.position.y = 0.05;
      pose.position.z = 1.90;
      step++;
      break;
    case 9:
      pose.position.y = 0.05;
      pose.position.z = 1.80;
      step++;
      break;
    case 10:
      pose.position.y = 0.05;
      pose.position.z = 1.70;
      step++;
      break;
    case 11:
      pose.position.y = 0.05;
      pose.position.z = 1.60;
      step++;
      break;
    case 12:
      pose.position.y = 0.05;
      pose.position.z = 1.50;
      step++;
      break;
    case 13:
      pose.position.y = 0.15;
      pose.position.z = 1.50;
      step++;
      break;
    case 14:
      pose.position.y = 0.25;
      pose.position.z = 1.50;
      step++;
      break;
    case 15:
      pose.position.y = 0.35;
      pose.position.z = 1.50;
      step++;
      break;
    case 16:
      pose.position.y = 0.45;
      pose.position.z = 1.50;
      step++;
      break;
    case 17:
      pose.position.y = 0.45;
      pose.position.z = 1.60;
      step++;
      break;
    case 18:
      pose.position.y = 0.45;
      pose.position.z = 1.70;
      step++;
      break;
    case 19:
      pose.position.y = 0.35;
      pose.position.z = 1.70;
      step++;
      break;
    default:
      break;
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_trajectory_node");
  ros::NodeHandle nh;

  std::string gz_topic, link_gz, link_rviz, ref_rviz, ref_gz;
  std::string bool_topic;
  ros::param::get("bool_topic", bool_topic);
  ros::param::get("gazebo_topic", gz_topic);
  ros::param::get("link_gazebo", link_gz);
  ros::param::get("link_rviz", link_rviz);
  ros::param::get("reference_gazebo", ref_gz);
  ros::param::get("reference_rviz", ref_rviz);

  ros::Duration(30.0).sleep();  // wait a 1/2 min to initialize

  ros::Publisher pub = nh.advertise<gazebo_msgs::LinkState>(gz_topic, 1);
  ros::Publisher pub_bool = nh.advertise<std_msgs::Bool>(bool_topic, 1);

  geometry_msgs::Pose central;
  central.position.x = 1.25;
  central.position.y = 0.25;
  central.position.z = 1.50;
  central.orientation.x = 0.0;
  central.orientation.y = 0.0;
  central.orientation.z = 1.0;
  central.orientation.w = 0.001;

  bool tri = true;
  bool squ = false;
  int step = 0;
  while (nh.ok())
  {
    gazebo_msgs::LinkState random_state;
    random_state.link_name = link_gz;
    random_state.reference_frame = ref_gz;

    // Define pose
    if (tri)
    {
      squ = false;
      tri = triangle(central, step);
      if (!tri)
      {
        squ = true;
      }
    }

    if (squ)
    {
      tri = false;
      squ = square(central, step);
      if (!squ)
      {
        tri = true;
      }
    }

    random_state.pose = central;

    // Define twist (gazebo msg)
    random_state.twist.angular.x = 0.0;
    random_state.twist.angular.y = 0.0;
    random_state.twist.angular.z = 0.0;
    random_state.twist.linear.x = 0.0;
    random_state.twist.linear.y = 0.0;
    random_state.twist.linear.z = 0.0;

    std_msgs::Bool msg;
    msg.data = true;
    pub_bool.publish(msg);
    pub.publish(random_state);
    sendTf(random_state.pose);
    ROS_INFO("Moving ball to (%f, %f, %f)", random_state.pose.position.x, random_state.pose.position.y,
             random_state.pose.position.z);
    ros::Duration(1.5).sleep();

    ros::spinOnce();
  }

  return 0;
}