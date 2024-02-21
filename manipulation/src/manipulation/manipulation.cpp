#include <manipulation/manipulation.h>

constexpr char LOGNAME[] = "moveit_task_constructor";
using namespace manipulation;

Manipulation::Manipulation()
{
}

Manipulation::~Manipulation()
{
  for(auto& task : task_plans)
  {
    task.second.reset();
  }
}

void 
Manipulation::setParameters(TaskParameters& params)
{
    parameters = params; 
}

// Service handler that receives message from executive to plan a task
bool Manipulation::handleManipulationPlanRequest(manipulation::GetManipulationPlan::Request &req, manipulation::GetManipulationPlan::Response &res)
{    
    std_srvs::Trigger srv; 
    
    ROS_INFO("[manipulation_node] Request: Update Planning Scene");
    
    if (update_planning_scene_service.call(srv)) {
        ROS_INFO("[manipulation_node] Response: Planning Scene updated");
    } else {
        res.manipulation_plan_response.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        ROS_ERROR("[manipulation_node] Response: Planning Scene failed to update for %s", req.manipulation_plan_request.task_name.c_str());
        return true;  
    }

    parameters.place_pose_ = req.manipulation_plan_request.place_pose;

    // PICK, PLACE, MOVE_ARM, OPEN_GRIPPER, CLOSE_GRIPPER, TODO: SHAKE, POUR, PUSH, PULL, OPEN_DOOR, CLOSE_DOOR
    parameters.task_type_ = req.manipulation_plan_request.task_type;
    parameters.object_name_ = req.manipulation_plan_request.object_name;
  
    // Use task name to label it within the list of tasks
    std::string task_name = req.manipulation_plan_request.task_name;
   
    // Add list of support surfaces to allow collisions 
    for (const auto& str : req.manipulation_plan_request.support_surfaces) {
        parameters.support_surfaces_.push_back(str); 
    }

    task_plans[task_name] = TaskFactory::createTask(parameters.task_type_, task_name.c_str());
    
    // Return failure if initialization fails
    if (!task_plans[task_name]->init(parameters))
    {
      res.manipulation_plan_response.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE; 
      return true; 
    }

    // Get if planning suceeded or failed
    res.manipulation_plan_response.error_code = task_plans[task_name]->plan();
    
    // Only add the solution if planning succeeded
    if(res.manipulation_plan_response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      task_plans[task_name]->getSolutionMsg(res.manipulation_plan_response.solution);   
    }

    return true;
}   
