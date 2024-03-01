#ifndef PICKTASK_H
#define PICKTASK_H

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
#include <actionlib/server/simple_action_server.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PlaceLocation.h>

#include <tasks/task_parameters.h>
#include <manipulation/ManipulationPlanRequest.h>
#include <tasks/task_base.h>
#include <tasks/task_factory.h>

using namespace moveit::task_constructor;

class PickTask : public TaskBase
{
public:
  PickTask(const std::string &task_name, const ros::NodeHandle &nh);
  ~PickTask() = default;
  bool init(const TaskParameters &parameters);

private:
  Stage *attach_object_stage_;
};

#endif
