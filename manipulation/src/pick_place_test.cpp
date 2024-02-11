#include <iostream>
#include "manipulation/manipulation.h"
#include "tasks/pick_place_task.h"
#include "tasks/move_to_goal_task.h"
#include "tasks/open_close_gripper_task.h"
#include <manipulation/PlanPickPlaceAction.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <manipulation/manipulation_parameters.h>

constexpr char LOGNAME[] = "pick place test";

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pick_place_test");
  ros::NodeHandle nh, pnh("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Duration(1.0).sleep();

  auto manipulation = std::make_unique<Manipulation>(); 
  manipulation->loadParameters(pnh);

  manipulation->plan_status = nh.advertise<moveit_task_constructor_msgs::ExecuteTaskSolutionActionResult>("/execute_task_solution/result", 1, true);
  manipulation->TestPickPlace(); 

  // Keep introspection alive
  //ros::waitForShutdown();
  return 0;
}
