#include <tasks/task_base.h>

TaskBase::TaskBase(const std::string& task_name) : task_name_(task_name)
{
    task_.reset(new moveit::task_constructor::Task(task_name));
    task_->loadRobotModel();
}

bool
TaskBase::execute()
{
    TASK_INFO("Executing solution trajectory");
    moveit_msgs::MoveItErrorCodes execute_result;
    task_->introspection().publishSolution(*task_->solutions().front());
    execute_result = task_->execute(*task_->solutions().front());
    // // If you want to inspect the goal message, use this instead:
    // actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction>
    // execute("execute_task_solution", true); execute.waitForServer();
    // moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
    // task_->solutions().front()->fillMessage(execute_goal.solution);
    // execute.sendGoalAndWait(execute_goal);
    // execute_result = execute.getResult()->error_code;
  
    if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_ERROR_STREAM("[" << task_name_.c_str() << "] Task execution failed and returned: " << execute_result.val);
      return false;
    }
  
    return true;

}

void 
TaskBase::getSolutionMsg(moveit_task_constructor_msgs::Solution& solution)
{
  task_->solutions().front()->appendTo(solution);
}

void 
TaskBase::preempt()
{
  task_->preempt();
}

bool
TaskBase::plan()
{
    TASK_INFO("Searching for task solutions");
    try
    { 
      task_->plan(1);
    }
    catch (InitStageException& e)
    { 
      ROS_ERROR_STREAM("[" << task_name_.c_str() << "] Initialization failed: " << e);
      return false;
    }
    if (task_->numSolutions() == 0)
    { 
      ROS_ERROR_STREAM("[" << task_name_.c_str() << "] Planning failed: ");
      return false;
    }

    publishSolution();

    return true;
}

void 
TaskBase::publishSolution()
{
    moveit_task_constructor_msgs::Solution solution;
    getSolutionMsg(solution);
  
    //ROS_INFO("%d", solution.sub_trajectory[0].trajectory.joint_trajectory.size());
    task_->introspection().publishSolution(*task_->solutions().front());
       
}

void
TaskBase::exposeTo(SerialContainer& container, const std::initializer_list<std::string>& properties)
{
    task_->properties().exposeTo(container.properties(), properties);
}

void
TaskBase::setProperty(const std::string& group, const std::string& name)
{
    task_->setProperty(group, name); 
}

void 
TaskBase::addStageToTask(Stage::pointer&& stage)
{
    task_->add(std::move(stage));
}

const robot_model::JointModelGroup* 
TaskBase::getJointModelGroup(const std::string& group_name)
{
    return task_->getRobotModel()->getJointModelGroup(group_name);
}

std::vector<std::string>
TaskBase::getLinkModelNamesWithCollisionGeometry(const std::string& group_name)
{
    return task_->getRobotModel()->getJointModelGroup(group_name)->getLinkModelNamesWithCollisionGeometry(); 
}

void
TaskBase::TASK_INFO(const std::string& info)
{
    ROS_INFO("[%s]: %s", task_name_.c_str(), info.c_str());
}

bool
TaskBase::initTask()
{
    try {
        task_->init();
    } catch (InitStageException& e) {
        ROS_ERROR_STREAM("[" << task_name_.c_str() << "] Initialization failed: " << e);
        return false;
    }
    return true;
}
