#include <tasks/wave_task.h>

const bool registered = TaskFactory::registerTask(
    manipulation::ManipulationPlanRequest::WAVE,
    [](const std::string &taskName, const ros::NodeHandle &nh) -> std::unique_ptr<TaskBase>
    {
      return std::make_unique<WaveTask>(taskName, nh);
    });

// static const bool registered = TaskFactory::registerTask(manipulation::ManipulationPlanRequest::WAVE, [](const std::string& taskName) -> TaskBase* { return new WaveTask(taskName);});

using namespace manipulation;

WaveTask::WaveTask(const std::string &task_name, const ros::NodeHandle &nh) : TaskBase(task_name, nh)
{
  current_state_stage_ = nullptr;
}

bool WaveTask::init(const TaskParameters &parameters)
{

  TASK_INFO("Initializing mtc pipeline");
  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
  // sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
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
    _current_state->setTimeout(10);

    // Verify that object is not attached for picking and if object is attached for placing
    auto applicability_filter =
        std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
    applicability_filter->setPredicate([&](const SolutionBase &s, std::string &comment)
                                       {
      s.start()->scene()->printKnownObjects(std::cout);

        if (s.start()->scene()->getCurrentState().hasAttachedBody(parameters.object_name_))
        {
          comment = "object with id '" + parameters.object_name_ + "' is attached and must be placed down first.";
          return false;
        }
      return true; });

    current_state_stage_ = applicability_filter.get();
    addStageToTask(std::move(applicability_filter));
  }
  {
    // Open hand stage
    {
      auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
      stage->setGroup(parameters.hand_group_name_);
      stage->setGoal(parameters.hand_open_pose_);
      addStageToTask(std::move(stage));
    }

    geometry_msgs::PoseStamped hand_wave_start_pose;
    hand_wave_start_pose.header.frame_id = parameters.base_frame_;
    hand_wave_start_pose.pose.position.x = 0.042;
    hand_wave_start_pose.pose.position.y = 0.384;
    hand_wave_start_pose.pose.position.z = 1.626;
    hand_wave_start_pose.pose.orientation.y = 0.7068907;
    hand_wave_start_pose.pose.orientation.w = -0.7073228;

    geometry_msgs::PoseStamped hand_wave_end_pose;
    hand_wave_end_pose.header.frame_id = parameters.base_frame_;
    hand_wave_end_pose.pose.position.x = 0.047;
    hand_wave_end_pose.pose.position.y = 0.645;
    hand_wave_end_pose.pose.position.z = 1.622;
    hand_wave_end_pose.pose.orientation.y = 0.7068907;
    hand_wave_end_pose.pose.orientation.w = -0.7073228;

    std::vector<geometry_msgs::PoseStamped> hand_poses{
        hand_wave_start_pose,
        hand_wave_end_pose,
        hand_wave_start_pose,
        hand_wave_end_pose,
        hand_wave_start_pose,
        hand_wave_end_pose,
        hand_wave_start_pose,
        hand_wave_end_pose};

    {
      for (int i = 0; i < hand_poses.size(); i++)
      {
        std::string goal = "Goal " + std::to_string(i);
        auto stage = std::make_unique<stages::MoveTo>(goal, sampling_planner);
        stage->properties().configureInitFrom(Stage::PARENT, {"group"});
        stage->setGoal(hand_poses[i]);
        addStageToTask(std::move(stage));
      }
    }
    {
      auto stage = std::make_unique<stages::MoveTo>("home", sampling_planner);
      stage->setGroup(parameters.hand_group_name_);
      stage->setGoal(parameters.arm_ready_pose_);
      addStageToTask(std::move(stage));
    }
  }
  return initTask();
}
