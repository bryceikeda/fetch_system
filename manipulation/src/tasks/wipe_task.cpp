#include <tasks/wipe_task.h>

const bool registered = TaskFactory::registerTask(
  manipulation::ManipulationPlanRequest::WIPE,
  [](const std::string& taskName) -> std::unique_ptr<TaskBase> {
      return std::make_unique<WipeTask>(taskName);
  }
);

WipeTask::WipeTask(const std::string& task_name) : TaskBase(task_name)
{
  current_state_stage_ = nullptr;
}

bool WipeTask::init(const TaskParameters& parameters)
{
    TASK_INFO("Initializing mtc pipeline");  
    auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
    //sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
    sampling_planner->setPlannerId("RRTConnectkConfigDefault");

    // Cartesian planner
    auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(.2);
    cartesian_planner->setMaxAccelerationScalingFactor(.2);
    cartesian_planner->setStepSize(.01);

    // Set task properties, names used for the specific arm group by robot
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

  geometry_msgs::PoseStamped gripper_pose;
  gripper_pose.header.frame_id = parameters.base_frame_;
  gripper_pose.pose.position.x = 0.042;
  gripper_pose.pose.position.y = 0.384;
  gripper_pose.pose.position.z = .5;
  gripper_pose.pose.orientation.y = 0;
  gripper_pose.pose.orientation.w = 1;

  std::string goal = "Goal";
  auto stage = std::make_unique<stages::MoveTo>(goal, sampling_planner);
  stage->properties().configureInitFrom(Stage::PARENT, { "group" });
  stage->setGoal(gripper_pose);
  addStageToTask(std::move(stage));

    return initTask();
}
