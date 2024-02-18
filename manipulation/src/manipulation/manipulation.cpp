#include <manipulation/manipulation.h>

constexpr char LOGNAME[] = "moveit_task_constructor";
using namespace manipulation;

Manipulation::Manipulation()
{
}

void 
Manipulation::setParameters(TaskParameters& params)
{
    parameters = params; 
}

void Manipulation::TestPickPlace()
{
    parameters.task_type_ = manipulation::ManipulationPlanRequest::PICK;

    ROS_INFO_STREAM(parameters.place_pose_ << "\n" << parameters.object_name_);


    auto pick = std::unique_ptr<TaskBase>(TaskFactory::createTask(manipulation::ManipulationPlanRequest::PICK, "pick"));

    if (!pick->init(parameters))
    {
      ROS_INFO_NAMED(LOGNAME, "Initialization failed");
      return;
    }

    if (pick->plan())
    {
        ROS_INFO_NAMED(LOGNAME, "Planning succeded");
        ros::Duration(3.0).sleep();

        if (pick->execute())
        {
          ROS_INFO_NAMED(LOGNAME, "Execution complete");
        }
        else
        {
           ROS_INFO_NAMED(LOGNAME, "Execution failed");
        }
    }
    else
    {
      ROS_INFO_NAMED(LOGNAME, "Planning failed");
    }
    
    parameters.task_type_ = manipulation::ManipulationPlanRequest::PICK;

    ROS_INFO_STREAM(parameters.place_pose_ << "\n" << parameters.object_name_);


    auto place = std::unique_ptr<TaskBase>(TaskFactory::createTask(manipulation::ManipulationPlanRequest::PLACE, "place"));

   
    if (!place->init(parameters))
    {
      ROS_INFO_NAMED(LOGNAME, "Initialization failed");
      return;
    }

    if (place->plan())
    {
        ROS_INFO_NAMED(LOGNAME, "Planning succeded");
        ros::Duration(3.0).sleep();

        if (place->execute())
        {
          ROS_INFO_NAMED(LOGNAME, "Execution complete");
        }
        else
        {
           ROS_INFO_NAMED(LOGNAME, "Execution failed");
        }
    }
    else
    {
      ROS_INFO_NAMED(LOGNAME, "Planning failed");
    }

}

// Service handler that receives message from executive to plan a task
bool Manipulation::handleManipulationPlanRequest(manipulation::GetManipulationPlan::Request &req, manipulation::GetManipulationPlan::Response &res)
{    
    std_srvs::Trigger srv; 
    if (update_planning_scene_service.call(srv)) {
        ROS_INFO("Planning scene updated.");
    } else {
        res.manipulation_plan_response.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        ROS_ERROR("[%s] Planning scene failed to update", req.manipulation_plan_request.task_name.c_str());
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
        std::cout << str << std::endl; 
    }

    auto task = TaskFactory::createTask(parameters.task_type_, task_name.c_str());
    if (!task->init(parameters))
    {
      res.manipulation_plan_response.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE; 
      return true; 
    }

    if (!task->plan())
    {
      res.manipulation_plan_response.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED; 
      return true; 
    }

    task->getSolutionMsg(res.manipulation_plan_response.solution); 
   
    res.manipulation_plan_response.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS; 

    // Publish Solution 
    task->publishSolution();
    return true;
}   
