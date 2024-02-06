#ifndef MANIPULATIONPARAMETERS_H
#define MANIPULATIONPARAMETERS_H

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/RobotState.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

struct ManipulationParameters
{
  short task_type_;

  // planning group properties
  std::string arm_group_name_;
  std::string eef_name_;
  std::string hand_group_name_;
  std::string hand_frame_;
  std::string base_frame_;

  // predefined poses in srdf
  std::string hand_open_pose_;
  std::string hand_close_pose_;
  std::string arm_home_pose_;

  // object + surface
  std::vector<std::string> support_surfaces_;
  std::string object_name_;

  // Pick metrics
  Eigen::Isometry3d grasp_frame_transform_;

  // Place metrics
  geometry_msgs::Pose place_pose_;

  // Pick and Place Metrics
  double approach_object_min_dist_;
  double approach_object_max_dist_;
  double lift_object_min_dist_;
  double lift_object_max_dist_;
  double place_surface_offset_;

  geometry_msgs::Pose object_offset_;

  // For move to goal, either specify joints or link pose
  bool use_joint_positions_;
  // Joint positions
  std::vector<moveit_msgs::RobotState> robot_states_;
  // Link poses
  std::vector<geometry_msgs::PoseStamped> hand_poses_;
  
  // Open close task
  bool open_hand_;
};

#endif
