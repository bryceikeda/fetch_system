#include <iostream>
#include "manipulation/manipulation.h"
#include "tasks/pick_place_task.h"
#include "tasks/move_to_goal_task.h"
#include "tasks/open_close_gripper_task.h"
#include <manipulation/PlanPickPlaceAction.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <manipulation/manipulation_parameters.h>

constexpr char LOGNAME[] = "manipulation node";

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "manipulation_node");
    ros::NodeHandle nh, pnh("~");

    auto manipulation = std::make_unique<Manipulation>(); 
    manipulation->loadParameters(pnh);
    manipulation->plan_status = nh.advertise<moveit_task_constructor_msgs::ExecuteTaskSolutionActionResult>("/execute_task_solution/result", 1, true);
    
    manipulation->get_manipulation_plan_service = nh.advertiseService("get_manipulation_plan", &Manipulation::handleManipulationPlanRequest, manipulation.get());
 
	ros::Duration(5.0).sleep();
	ros::Rate loop_rate(40); 
	
    while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep(); 
	}

    // Keep introspection alive
    ros::waitForShutdown();
    return 0;
}
