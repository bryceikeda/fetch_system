#include <tasks/task_base.h>

//Initializes the TaskBase with a specified task name.
TaskBase::TaskBase(const std::string& task_name) : task_name_(task_name)
{
    // Create a new Task instance and load the robot model associated with it.
    task_.reset(new moveit::task_constructor::Task());
    // Setting this names the stages the task, not the solution which is used
    // to publish the solution on topic
    task_->stages()->setName(task_name);
    task_->loadRobotModel();
}

TaskBase::~TaskBase()
{

}

double
TaskBase::getHeightOffsetForSurface(const std::string& object_name, const std::string& place_surface_name, const double place_surface_offset)
{
    moveit::planning_interface::PlanningSceneInterface psi;
    moveit_msgs::CollisionObject place_surface_co = psi.getObjects()[place_surface_name];
    moveit_msgs::CollisionObject object_co = psi.getAttachedObjects({ object_name })[object_name].object;
    
    double place_offset_from_surface_origin = 0.0;

    // Return and empty message if surface can't be found
    if (place_surface_co.primitives.empty() && place_surface_co.meshes.empty())
    {
        ROS_ERROR_STREAM("[" << task_name_.c_str() << "] Place surface with id '" << object_name << "' cannot be found");
        return 0;
    }
    
    // Return and empty message if object not attached
    if (object_co.primitives.empty() && object_co.meshes.empty())
    {
        ROS_ERROR_STREAM("[" << task_name_.c_str() << "] Object with id '" << object_name << "' is not attached, so it cannot be placed");
        return 0;
    }

    // Get the offset of the place surface
    if(place_surface_co.primitives[0].type == shape_msgs::SolidPrimitive::BOX)
    {
        // Whole surface object origin -> center of surface collision object + half the surface collision object hight
        place_offset_from_surface_origin += place_surface_co.primitive_poses[0].position.z + 0.5 * place_surface_co.primitives[0].dimensions[2];
    }
    else
    {
        place_offset_from_surface_origin += 0.5 * place_surface_co.primitives[0].dimensions[0];
    }

    // Get the offset of the object
    if(!object_co.meshes.empty())
    {
        place_offset_from_surface_origin += place_surface_offset;
    }
    else if(object_co.primitives[0].type == shape_msgs::SolidPrimitive::BOX)
    {
        place_offset_from_surface_origin += 0.5 * object_co.primitives[0].dimensions[2] + place_surface_offset;
    }
    else
    {
        place_offset_from_surface_origin += 0.5 * object_co.primitives[0].dimensions[0] + place_surface_offset;
    }

    return place_offset_from_surface_origin;
}


// Executes the planned task trajectory.
bool TaskBase::execute()
{
    TASK_INFO("Executing solution trajectory");

    // Publish the task solution for introspection.
    task_->introspection().publishSolution(*task_->solutions().front());

	actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction>

	execute("execute_task_solution", true); 
    execute.waitForServer();

	moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
    getSolutionMsg(execute_goal.solution);
	execute.sendGoalAndWait(execute_goal);

    moveit_msgs::MoveItErrorCodes execute_result = execute.getResult()->error_code;


    // Handle the execution result.
    if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        ROS_ERROR_STREAM("[" << task_name_.c_str() << "] Task execution failed and returned: " << execute_result.val);
        return false;
    }

    return true;
}

void TaskBase::getSolutionMsg(moveit_task_constructor_msgs::Solution& solution)
{
    task_->solutions().front()->toMsg(solution);
}

void TaskBase::preempt()
{
    task_->preempt();
}

moveit_msgs::MoveItErrorCodes TaskBase::plan(int max_solutions)
{
    TASK_INFO("Searching for task solutions");

    moveit_msgs::MoveItErrorCodes error_code = task_->plan(max_solutions);

    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        // Publish the task solution for external consumption.
        task_->introspection().publishSolution(*task_->solutions().front());
        TASK_INFO("Planning succeeded");
    }
    else{
        ROS_ERROR_STREAM("[" << task_name_.c_str() << "] Planning failed and returned: " << error_code.val);
    }
    
    return error_code;
}

// Makes specified properties available for serialization container.
void TaskBase::exposeTo(SerialContainer& container, const std::initializer_list<std::string>& properties)
{
    task_->properties().exposeTo(container.properties(), properties);
}

// Associates a property with a specific group and name.
void TaskBase::setProperty(const std::string& group, const std::string& name)
{
    task_->setProperty(group, name);
}

// Appends a stage to the task execution pipeline.
void TaskBase::addStageToTask(Stage::pointer&& stage)
{
    task_->add(std::move(stage));
}

// Retrieves the joint model group for a specified group name.
const robot_model::JointModelGroup* TaskBase::getJointModelGroup(const std::string& group_name)
{
    return task_->getRobotModel()->getJointModelGroup(group_name);
}

// Retrieves names of link models with collision geometry for a specified group.
std::vector<std::string> TaskBase::getLinkModelNamesWithCollisionGeometry(const std::string& group_name)
{
    return task_->getRobotModel()->getJointModelGroup(group_name)->getLinkModelNamesWithCollisionGeometry();
}

// Outputs informational messages related to the task.
void TaskBase::TASK_INFO(const std::string& info)
{
    ROS_INFO("[%s]: %s", task_name_.c_str(), info.c_str());
}

bool TaskBase::initTask()
{
    try
    {
        // Attempt to initialize the task.
        task_->init();
    }
    catch (InitStageException& e)
    {
        ROS_ERROR_STREAM("[" << task_name_.c_str() << "] Initialization failed: " << e);
        return false;
    }

    return true;
}
