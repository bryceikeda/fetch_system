#include <tasks/pick_place_task.h>

constexpr char LOGNAME[] = "pick and place task";

using namespace manipulation;

PickPlaceTask::PickPlaceTask(const std::string& task_name) : task_name_(task_name)
{
  task_.reset();
  task_.reset(new moveit::task_constructor::Task(task_name_));
  task_->loadRobotModel();


  current_state_stage_ = nullptr;
  attach_object_stage_ = nullptr;
}

bool PickPlaceTask::init(const ManipulationParameters& parameters)
{
  ROS_INFO_NAMED(LOGNAME, "Initializing pick and place object pipeline");
  // Create new Task and call it pick_place_task with task_name_name
  // Task is the root of a tree of stages, default serial container wraps all stages
  Task& t = *task_;

  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
  //sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
  sampling_planner->setPlannerId("RRTConnectkConfigDefault");

  // Cartesian planner
  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(.2);
  cartesian_planner->setMaxAccelerationScalingFactor(.2);
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

      if (parameters.task_type_ == PickPlace::PICK_ONLY ||
          parameters.task_type_ == PickPlace::PICK_AND_PLACE)
      {
        if (s.start()->scene()->getCurrentState().hasAttachedBody(parameters.object_name_))
        {
          comment = "object with id '" + parameters.object_name_ + "' is already attached and cannot be picked";
          return false;
        }
      }
      else if (parameters.task_type_ == PickPlace::PLACE_ONLY)
      {
        if (!s.start()->scene()->getCurrentState().hasAttachedBody(parameters.object_name_))
        {
          comment = "object with id '" + parameters.object_name_ + "' is not attached, so it cannot be placed";
          return false;
        }
      }
      return true;
    });

    current_state_stage_ = applicability_filter.get();
    t.add(std::move(applicability_filter));
  }
  if (parameters.task_type_ == PickPlace::PICK_ONLY ||
      parameters.task_type_ == PickPlace::PICK_AND_PLACE)
  {
    // Open hand stage
    {
      auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
      stage->setGroup(parameters.hand_group_name_);
      stage->setGoal(parameters.hand_open_pose_);
      t.add(std::move(stage));
    }

    // Move to pick stage
    {
      auto stage = std::make_unique<stages::Connect>(
          "move to pick", stages::Connect::GroupPlannerVector{ { parameters.arm_group_name_, sampling_planner } });
      stage->setTimeout(5.0);
      stage->properties().configureInitFrom(Stage::PARENT);
      t.add(std::move(stage));
    }

    // approach object
    {
      auto grasp = std::make_unique<SerialContainer>("pick object");
      t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
      grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

      /****************************************************
    ---- *               Approach Object                    *
       ***************************************************/
      {
        auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
        stage->properties().set("marker_ns", "approach_object");
        stage->properties().set("link", parameters.hand_frame_);
        stage->properties().configureInitFrom(Stage::PARENT, { "group" });
        stage->setMinMaxDistance(parameters.approach_object_min_dist_, parameters.approach_object_max_dist_);

        // Set hand forward direction
        geometry_msgs::Vector3Stamped vec;
        vec.header.frame_id = parameters.base_frame_;
        vec.vector.x = 1.0;
        stage->setDirection(vec);
        grasp->insert(std::move(stage));
      }

      /****************************************************
    ---- *               Generate Grasp Pose                *
       ***************************************************/
      {
        // Sample grasp pose
        auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
        stage->properties().configureInitFrom(Stage::PARENT);
        stage->properties().set("marker_ns", "grasp_pose");
        stage->setPreGraspPose(parameters.hand_open_pose_);
        stage->setObject(parameters.object_name_);
        stage->setAngleDelta(M_PI / 12); // pi/12
        stage->setMonitoredStage(current_state_stage_);  // Hook into current state

        // Compute IK
        auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(8);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(parameters.grasp_frame_transform_, parameters.hand_frame_);
        wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
        grasp->insert(std::move(wrapper));
      }

      /****************************************************
    ---- *               Allow Collision (hand object)   *
       ***************************************************/
      {
        auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
        stage->allowCollisions(parameters.object_name_,
                               t.getRobotModel()
                                   ->getJointModelGroup(parameters.hand_group_name_)
                                   ->getLinkModelNamesWithCollisionGeometry(),
                               true);
        grasp->insert(std::move(stage));
      }

      /****************************************************
    ---- *               Close Hand                      *
       ***************************************************/
      {
        auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
        stage->setGroup(parameters.hand_group_name_);
        stage->setGoal(parameters.hand_close_pose_);
        grasp->insert(std::move(stage));
      }

      /****************************************************
    .... *               Attach Object                      *
       ***************************************************/
      {
        auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
        stage->attachObject(parameters.object_name_, parameters.hand_frame_);
        attach_object_stage_ = stage.get();
        grasp->insert(std::move(stage));
      }

      /****************************************************
    .... *               Allow collision (object support)   *
       ***************************************************/
      {
        auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
        stage->allowCollisions({ parameters.object_name_ }, parameters.support_surfaces_, true);
        grasp->insert(std::move(stage));
      }

      /****************************************************
    .... *               Lift object                        *
       ***************************************************/
      {
        auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
        stage->properties().configureInitFrom(Stage::PARENT, { "group" });
        stage->setMinMaxDistance(parameters.lift_object_min_dist_, parameters.lift_object_max_dist_);
        stage->setIKFrame(parameters.hand_frame_);
        stage->properties().set("marker_ns", "lift_object");

        // Set upward direction
        geometry_msgs::Vector3Stamped vec;
        vec.header.frame_id = parameters.base_frame_;
        vec.vector.z = 1.0;
        stage->setDirection(vec);
        grasp->insert(std::move(stage));
      }

      /****************************************************
    .... *               Forbid collision (object support)  *
       ***************************************************/
      {
        auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
        stage->allowCollisions({ parameters.object_name_ }, parameters.support_surfaces_, false);
        grasp->insert(std::move(stage));
      }

      t.add(std::move(grasp));
    }
  }
  if (parameters.task_type_ == PickPlace::PLACE_ONLY ||
      parameters.task_type_ == PickPlace::PICK_AND_PLACE)
  {
    {
      auto stage = std::make_unique<stages::Connect>(
          "move to place", stages::Connect::GroupPlannerVector{ { parameters.arm_group_name_, sampling_planner } });
      stage->setTimeout(5.0);
      stage->properties().configureInitFrom(Stage::PARENT);
      t.add(std::move(stage));
    }

    {
      auto place = std::make_unique<SerialContainer>("place object");
      t.properties().exposeTo(place->properties(), { "eef", "hand", "group" });
      place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group" });

      /******************************************************
    ---- *          Lower Object                              *
       *****************************************************/
      {
        auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
        stage->properties().set("marker_ns", "lower_object");
        stage->properties().set("link", parameters.hand_frame_);
        stage->properties().configureInitFrom(Stage::PARENT, { "group" });
        stage->setMinMaxDistance(.03, .13);

        // Set downward direction
        geometry_msgs::Vector3Stamped vec;
        vec.header.frame_id = parameters.base_frame_;
        vec.vector.z = -1.0;
        stage->setDirection(vec);
        place->insert(std::move(stage));
      }

      /******************************************************
    ---- *          Generate Place Pose                       *
       *****************************************************/
      {
        // Generate Place Pose
        auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
        stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
        stage->properties().set("marker_ns", "place_pose");
        stage->setObject(parameters.object_name_);

        moveit_msgs::CollisionObject collision_object;
        moveit::planning_interface::PlanningSceneInterface psi;

        // If place only then get the object attached to gripper
        if (parameters.task_type_ == PickPlace::PLACE_ONLY)
        {
          collision_object = psi.getAttachedObjects({ parameters.object_name_ })[parameters.object_name_].object;
        }
        else
        {  // Get object from planning scene
          collision_object = psi.getObjects({ parameters.object_name_ })[parameters.object_name_];
          ROS_INFO_STREAM(collision_object);
        }

      
        // Set target pose
        geometry_msgs::PoseStamped p;

        p.header.frame_id = parameters.base_frame_;  // collision_object.header.frame_id;  // object_reference_frame was used before
        p.pose = parameters.place_pose_;
        p.pose.position.z += .03;

        // Take half the objects height and add that to the z position to properly place the object
        // if (collision_object.primitives[0].type == shape_msgs::SolidPrimitive::BOX)
        // {
        //   p.pose.position.z += 0.5 * collision_object.primitives[0].dimensions[2] + parameters.place_surface_offset_;
        // }
        // else
        // {
        //   p.pose.position.z += 0.5 * collision_object.primitives[0].dimensions[0] + parameters.place_surface_offset_;
        // }

        stage->setPose(p);

        if (parameters.task_type_ == PickPlace::PLACE_ONLY)
          stage->setMonitoredStage(current_state_stage_);
        if (parameters.task_type_ == PickPlace::PICK_AND_PLACE)
          stage->setMonitoredStage(attach_object_stage_);

        // Compute IK
        auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(8);
        wrapper->setIKFrame(parameters.grasp_frame_transform_, parameters.hand_frame_);
        wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
        place->insert(std::move(wrapper));
      }

      /******************************************************
    ---- *          Open Hand                              *
       *****************************************************/
      {
        auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
        stage->setGroup(parameters.hand_group_name_);
        stage->setGoal(parameters.hand_open_pose_);
        place->insert(std::move(stage));
      }

      /******************************************************
    ---- *          Forbid collision (hand, object)        *
       *****************************************************/
      {
        auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
        stage->allowCollisions(parameters.object_name_,
                               *t.getRobotModel()->getJointModelGroup(parameters.hand_group_name_), false);
        place->insert(std::move(stage));
      }

      /******************************************************
    ---- *          Detach Object                             *
       *****************************************************/
      {
        auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
        stage->detachObject(parameters.object_name_, parameters.hand_frame_);
        place->insert(std::move(stage));
      }

      /******************************************************
    ---- *          Retreat Motion                            *
       *****************************************************/
      {
        auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
        stage->properties().configureInitFrom(Stage::PARENT, { "group" });
        stage->setMinMaxDistance(.12, .25);
        stage->setIKFrame(parameters.hand_frame_);
        stage->properties().set("marker_ns", "retreat");
        geometry_msgs::Vector3Stamped vec;
        vec.header.frame_id = parameters.hand_frame_;
        vec.vector.x = -1.0;
        stage->setDirection(vec);
        place->insert(std::move(stage));
      }

      // Add place container to task
      t.add(std::move(place));
    }
      /******************************************************
     *                                                    *
     *          Move to Home                              *
     *                                                    *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setGoal(parameters.arm_home_pose_);
      stage->restrictDirection(stages::MoveTo::FORWARD);
      t.add(std::move(stage));
    }
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

bool PickPlaceTask::plan()
{
  ROS_INFO_NAMED(LOGNAME, "Start searching for task solutions");
  try
  {
    task_->plan(1);  // TODO(karolyartur): parameterize
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
  moveit_task_constructor_msgs::Solution solution; 
  getSolutionMsg(solution); 

//ROS_INFO("%d", solution.sub_trajectory[0].trajectory.joint_trajectory.size());
  task_->introspection().publishSolution(*task_->solutions().front());
  return true;
}

bool PickPlaceTask::preempt()
{
  task_->preempt();
  return true;
}

void PickPlaceTask::getSolutionMsg(moveit_task_constructor_msgs::Solution& solution)
{
  task_->solutions().front()->appendTo(solution);
}

bool PickPlaceTask::execute()
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
