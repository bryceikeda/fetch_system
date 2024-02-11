#include <tasks/move_to_goal_task.h>

constexpr char LOGNAME[] = "move to task";

MoveToGoalTask::MoveToGoalTask(const std::string& task_name) : TaskBase(task_name)
{
  current_state_stage_ = nullptr;
}

bool MoveToGoalTask::init(const ManipulationParameters& parameters)
{
  TASK_INFO("Initializing move to pipeline");
  
  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

  // Set task properties, names used for the specific arm group by robot
  // Set task properties
  setProperty("group", parameters.arm_group_name_);
  setProperty("eef", parameters.eef_name_);
  setProperty("hand", parameters.hand_group_name_);
  setProperty("ik_frame", parameters.hand_frame_);
  
  /****************************************************
   *                                                  *
   *               Current State                      *
   *                                                  *
   ***************************************************/
  {
    auto _current_state = std::make_unique<stages::CurrentState>("current state");
    addStageToTask(std::move(_current_state));
  }

  if(parameters.use_joint_positions_){
    for (int i = 0; i < parameters.robot_states_.size(); i++)
    {
      std::string goal = "Goal " + std::to_string(i);
      auto stage = std::make_unique<stages::MoveTo>(goal, sampling_planner);
      stage->setGroup(parameters.arm_group_name_);
      stage->setGoal(parameters.robot_states_[i]);
      addStageToTask(std::move(stage));
    }
  }
  else{
    for (int i = 0; i < parameters.hand_poses_.size(); i++)
    {
      std::string goal = "Goal " + std::to_string(i);
      auto stage = std::make_unique<stages::MoveTo>(goal, sampling_planner);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });

      stage->setGoal(parameters.hand_poses_[i]);
      addStageToTask(std::move(stage));
    }
  }

  return initTask();
}
