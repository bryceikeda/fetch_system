#include <tasks/open_close_gripper_task.h>

constexpr char LOGNAME[] = "Open or close hand ask";

OpenCloseGripperTask::OpenCloseGripperTask(const std::string& task_name) : task_name_(task_name)
{
  task_.reset();
  task_.reset(new moveit::task_constructor::Task(task_name_));
  task_->loadRobotModel();
  current_state_stage_ = nullptr;
}

bool OpenCloseGripperTask::init(const ManipulationParameters& parameters)
{
  ROS_INFO_NAMED(LOGNAME, "Initializing move to pipeline");
  // Create new Task and call it pick_place_task with task_name_name
  // Task is the root of a tree of stages, default serial container wraps all stages
  Task& t = *task_;

  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

  // Cartesian planner
  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  // Set task properties, names used for the specific arm group by robot
  // Set task properties
  t.setProperty("group", parameters.arm_group_name_);
  t.setProperty("eef", parameters.eef_name_);
  t.setProperty("hand", parameters.hand_group_name_);
  t.setProperty("ik_frame", parameters.hand_frame_);

  /****************************************************
   *                                                  *
   *               Current State                      *
   *                                                  *
   ***************************************************/
  {
    auto _current_state = std::make_unique<stages::CurrentState>("current state");
    _current_state->setTimeout(10);

    // Verify that object is not attached for picking and if object is attached for placing
    auto applicability_filter =
        std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
    applicability_filter->setPredicate([&](const SolutionBase& s, std::string& comment) {
      s.start()->scene()->printKnownObjects(std::cout);
      std::vector<const moveit::core::AttachedBody*> attached_bodies;
      s.start()->scene()->getCurrentState().getAttachedBodies(attached_bodies);
      if (attached_bodies.size() != 0)
      {
        comment = "Object attached to hand, use pick_place_task instead";
        return false; 
      }
      return true;
    });

    current_state_stage_ = applicability_filter.get();
    t.add(std::move(applicability_filter));
  }

  if (parameters.open_hand_)
  {
    /******************************************************
  ---- *          Open Hand                              *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
      stage->setGroup(parameters.hand_group_name_);
      stage->setGoal(parameters.hand_open_pose_);
      t.add(std::move(stage));
    }
  }
  else
  {
      auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
      stage->setGroup(parameters.hand_group_name_);
      stage->setGoal(parameters.hand_close_pose_);
      t.add(std::move(stage));
  }

  try
  {
    t.init();
  }
  catch (InitStageException& e)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Initialization failed: " << e);
    return false;
  }

  return true;
}

bool OpenCloseGripperTask::plan()
{
  ROS_INFO_NAMED(LOGNAME, "Start searching for task solutions");
  try
  {
    task_->plan(1);
  }
  catch (InitStageException& e)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Initialization failed: " << e);
    return false;
  }
  if (task_->numSolutions() == 0)
  {
    ROS_ERROR_NAMED(LOGNAME, "Planning failed");
    return false;
  }
  return true;
}

bool OpenCloseGripperTask::preempt()
{
  task_->preempt();
  return true;
}

void OpenCloseGripperTask::getSolutionMsg(moveit_task_constructor_msgs::Solution& solution)
{
  task_->solutions().front()->appendTo(solution);
}

bool OpenCloseGripperTask::execute()
{
  ROS_INFO_NAMED(LOGNAME, "Executing solution trajectory");
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
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Task execution failed and returned: " << execute_result.val);
    return false;
  }

  return true;
}
