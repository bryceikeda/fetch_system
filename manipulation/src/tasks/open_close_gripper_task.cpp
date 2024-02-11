#include <tasks/open_close_gripper_task.h>

constexpr char LOGNAME[] = "Open or close hand ask";

OpenCloseGripperTask::OpenCloseGripperTask(const std::string& task_name) : TaskBase(task_name)
{
  current_state_stage_ = nullptr;
}

bool OpenCloseGripperTask::init(const ManipulationParameters& parameters)
{
  TASK_INFO("Initializing move to pipeline");

  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

  // Cartesian planner
  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

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
    addStageToTask(std::move(applicability_filter));
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
      addStageToTask(std::move(stage));
    }
  }
  else
  {
      auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
      stage->setGroup(parameters.hand_group_name_);
      stage->setGoal(parameters.hand_close_pose_);
      addStageToTask(std::move(stage));
  }

  return initTask();
}
