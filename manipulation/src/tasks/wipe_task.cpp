#include <tasks/wipe_task.h>

const bool registered = TaskFactory::registerTask(
    manipulation::ManipulationPlanRequest::WIPE,
    [](const std::string &taskName, const ros::NodeHandle &nh) -> std::unique_ptr<TaskBase>
    {
      return std::make_unique<WipeTask>(taskName, nh);
    });

WipeTask::WipeTask(const std::string &task_name, const ros::NodeHandle &nh) : TaskBase(task_name, nh)
{
  current_state_stage_ = nullptr;
}

bool WipeTask::init(const TaskParameters &parameters)
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
  Stage *attach_object_stage_;
  {
    auto _current_state = std::make_unique<stages::CurrentState>("current state");
    _current_state->setTimeout(10);

    // Verify that object is not attached for picking and if object is attached for placing
    auto applicability_filter =
        std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
    applicability_filter->setPredicate([&](const SolutionBase &s, std::string &comment)
                                       {
        s.start()->scene()->printKnownObjects(std::cout);

          if (!s.start()->scene()->getCurrentState().hasAttachedBody(parameters.object_name_))
          {
            comment = "object with id '" + parameters.object_name_ + "' is not attached, so it cannot be placed";
            return false;
          }
        return true; });

    current_state_stage_ = applicability_filter.get();
    addStageToTask(std::move(applicability_filter));
  }
  // approach object
  {
    auto wipe_table = std::make_unique<SerialContainer>("Move to Wipe");
    exposeTo(*wipe_table, {"eef", "hand", "group", "ik_frame"});
    wipe_table->properties().configureInitFrom(Stage::PARENT, {"eef", "hand", "group", "ik_frame"});

    /****************************************************
    ---- *               Allow Collision (hand object)   *
    ***************************************************/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,surface)");
      stage->allowCollisions(parameters.object_name_, parameters.target_name_, true);
      attach_object_stage_ = stage.get();
      wipe_table->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<stages::MoveTo>("move to wipe start", sampling_planner);
      stage->setTimeout(5.0);
      stage->setIKFrame(parameters.grasp_frame_transforms_.find("vertical_grasp_frame_transform")->second, parameters.hand_frame_);
      stage->properties().configureInitFrom(Stage::PARENT);

      geometry_msgs::PoseStamped place_pose;
      place_pose.header.frame_id = parameters.target_name_ + "/" + "center";

      place_pose.pose = parameters.place_pose_;
      place_pose.pose.orientation.w = 1;
      place_pose.pose.position.z = getHeightOffsetForSurface(parameters.object_name_, parameters.target_name_, parameters.place_surface_offset_);
      stage->setGoal(place_pose);
      wipe_table->insert(std::move(stage));
    }
    addStageToTask(std::move(wipe_table));
  }
  {
    addStageToTask(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "right")));
    addStageToTask(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "right")));
    addStageToTask(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "right")));
    addStageToTask(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "left")));
    addStageToTask(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "left")));
    addStageToTask(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "left")));
    addStageToTask(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "left")));
    addStageToTask(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "left")));
    addStageToTask(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "left")));
    addStageToTask(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "right")));
    addStageToTask(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "right")));
    addStageToTask(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "right")));
  }
  return initTask();
}

std::unique_ptr<SerialContainer> WipeTask::moveDiagonal(const std::string &group, const TaskParameters &parameters, const std::string &direction)
{
  auto c = std::make_unique<SerialContainer>("Diagonal Path " + direction);
  c->setProperty("group", group);

  auto cartesian = std::make_shared<solvers::CartesianPath>();
  auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

  double up = 0.05;
  double down = -0.05;
  double dir = (direction == "left") ? 0.05 : -0.05;

  auto createDiagonalStage = [&](const std::string &name, double x, double y)
  {
    auto stage = std::make_unique<stages::MoveRelative>(name, cartesian);
    stage->properties().configureInitFrom(Stage::PARENT, {"group"});
    geometry_msgs::Vector3Stamped direction_vec;
    direction_vec.header.frame_id = parameters.base_frame_;
    direction_vec.vector.x = x;
    direction_vec.vector.y = y;
    stage->setDirection(direction_vec);
    c->insert(std::move(stage));
  };

  createDiagonalStage("diagonal upward", up, dir);
  createDiagonalStage("diagonal downward", down, dir);

  return c;
}