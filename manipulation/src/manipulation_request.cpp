#include <iostream>
#include "manipulation/manipulation.h"
#include "tasks/pick_place_task.h"
#include "tasks/move_to_goal_task.h"
#include "tasks/open_close_gripper_task.h"
#include <manipulation/PlanPickPlaceAction.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <manipulation/manipulation_parameters.h>
#include <manipulation/GetManipulationPlan.h>
#include <manipulation/ManipulationPlanRequest.h>

constexpr char LOGNAME[] = "manipulation request";

using namespace manipulation;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "manipulation_request");
  ros::NodeHandle nh, pnh("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Duration(1.0).sleep();

  GetManipulationPlan get_plan;
  get_plan.request.manipulation_plan_request.task_type = ManipulationPlanRequest::PICK_AND_PLACE; 

  get_plan.request.manipulation_plan_request.object_name = "demo_cube";
  get_plan.request.manipulation_plan_request.surface_name = "table1";

  geometry_msgs::Pose place_pose; 
  place_pose.position.x = .6;
  place_pose.position.y = .2;
  place_pose.position.z = .725;
  place_pose.orientation.w = 1.0;  
  
  ros::ServiceClient client = nh.serviceClient<manipulation::GetManipulationPlanRequest>("get_manipulation_plan");

  if (client.call(get_plan))
  {
    ROS_INFO("received");
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }
 
//  manipulation->plan_status = nh.advertise<moveit_task_constructor_msgs::ExecuteTaskSolutionActionResult>("/execute_task_solution/result", 1, true);


  // Keep introspection alive
  //ros::waitForShutdown();
  return 0;
}
