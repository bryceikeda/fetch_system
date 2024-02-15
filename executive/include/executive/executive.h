#ifndef EXECUTIVE_H
#define EXECUTIVE_H

#include <ros/ros.h>
#include <manipulation/GetManipulationPlan.h>
#include <manipulation/ManipulationPlanRequest.h>
#include <manipulation/ManipulationPlanResponse.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

class Executive
{
public:
    Executive();

    // Fixing the method signature and parameters
    ros::ServiceClient manipulation_plan_client;
  

private:
};

#endif
