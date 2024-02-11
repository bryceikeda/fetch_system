#ifndef MANIPULATION_H
#define MANIPULATION_H

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>
#include <moveit_task_constructor_msgs/Solution.h>
#include <manipulation/PlanPickPlaceAction.h>
#include <manipulation/PlanPickPlaceGoal.h>
#include <actionlib/server/simple_action_server.h>
#include "manipulation/PickPlace.h"
#include <manipulation/manipulation_parameters.h>
#include "tasks/pick_place_task.h"
#include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.h>
#include <manipulation/GetManipulationPlan.h>
#include <manipulation/ManipulationPlanRequest.h>
#include <tasks/task_base.h>
using namespace moveit::task_constructor;

typedef actionlib::SimpleActionServer<manipulation::PlanPickPlaceAction> PickPlaceServer;

class Manipulation
{
public:
  Manipulation();
  ~Manipulation() = default;

  void executePickPlaceCallback(const manipulation::PlanPickPlaceGoalConstPtr& goal);
  void preemptPickPlaceCallback();
  void planRobotActionCallback(const manipulation::PickPlaceConstPtr& msg);
  void setParameters(ManipulationParameters parameters);
  void loadParameters(const ros::NodeHandle& pnh_);

  bool handleManipulationPlanRequest(manipulation::GetManipulationPlan::Request &req, manipulation::GetManipulationPlan::Response &res);

  void TestPickPlace(); 
  ros::Publisher plan_status;
  ros::ServiceServer get_manipulation_plan_service;
private:
  void initializeServer();
  std::unique_ptr<PickPlaceServer> pick_place_server_;

  // Which transform to use
  int transform_orientation;
  std::vector<double> diagonal_frame_transform {0.02, 0, 0.0, 0, 3.9275, 0.0};  
  std::vector<double> horizontal_frame_transform {0.02, 0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> vertical_frame_transform {0.02, 0, 0.0,  0, 4.713, 0.0};

  bool preemt_requested_;
  static constexpr char LOGNAME[]{ "manipulation" };

  ManipulationParameters parameters; 
  moveit_task_constructor_msgs::Solution current_solution;  
  
  ros::NodeHandle pnh_;

};
#endif
