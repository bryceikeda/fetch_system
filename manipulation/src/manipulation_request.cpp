#include <iostream>
#include "manipulation/manipulation.h"
#include <manipulation/GetManipulationPlan.h>
#include <manipulation/ManipulationPlanRequest.h>
#include <actionlib/client/simple_action_client.h>

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
  get_plan.request.manipulation_plan_request.task_type = ManipulationPlanRequest::DANCE; 

  get_plan.request.manipulation_plan_request.object_name = "demo_cube";
  get_plan.request.manipulation_plan_request.support_surfaces = {"table1"};

  get_plan.request.manipulation_plan_request.task_name = "pick_and_place";

  geometry_msgs::Pose place_pose; 
  place_pose.position.x = .6;
  place_pose.position.y = .11;
  place_pose.position.z = .742311;
  place_pose.orientation.w = 1.0;  
 
  get_plan.request.manipulation_plan_request.place_pose = place_pose; 

  ros::ServiceClient client = nh.serviceClient<manipulation::GetManipulationPlanRequest>("get_manipulation_plan");

  actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction>
  execute("execute_task_solution", true); execute.waitForServer();
  
  moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;

  if (client.call(get_plan))
  {
    ROS_INFO("Received");
    execute_goal.solution = get_plan.response.manipulation_plan_response.solution;
    execute.sendGoalAndWait(execute_goal);
    moveit_msgs::MoveItErrorCodes execute_result = execute.getResult()->error_code;
    ROS_INFO("Executing Plan");
  }
  else
  {
    ROS_ERROR("Failed to call service get_manipulation_plan");
    return 1;
  }
 
//  manipulation->plan_status = nh.advertise<moveit_task_constructor_msgs::ExecuteTaskSolutionActionResult>("/execute_task_solution/result", 1, true);


  // Keep introspection alive
  ros::waitForShutdown();
  return 0;
}
