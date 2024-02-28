#include <tasks/place_task.h>

const bool registered = TaskFactory::registerTask(
      manipulation::ManipulationPlanRequest::PLACE,
      [](const std::string& taskName) -> std::unique_ptr<TaskBase> {
          return std::make_unique<PlaceTask>(taskName);
      }
  );

//static const bool registered = TaskFactory::registerTask(manipulation::ManipulationPlanRequest::PLACE, [](const std::string& taskName) -> TaskBase* { return new PlaceTask(taskName);}); 

using namespace manipulation;

PlaceTask::PlaceTask(const std::string& task_name) : TaskBase(task_name)
{
  current_state_stage_ = nullptr;
  attach_object_stage_ = nullptr;
}

bool PlaceTask::init(const TaskParameters& parameters)
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
    _current_state->setTimeout(10);

    // Verify that object is not attached for picking and if object is attached for placing
    auto applicability_filter =
        std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
    applicability_filter->setPredicate([&](const SolutionBase& s, std::string& comment) {
      s.start()->scene()->printKnownObjects(std::cout);

        if (!s.start()->scene()->getCurrentState().hasAttachedBody(parameters.object_name_))
        {
          comment = "object with id '" + parameters.object_name_ + "' is not attached, so it cannot be placed";
          return false;
        }
      return true;
    });

    current_state_stage_ = applicability_filter.get();
    addStageToTask(std::move(applicability_filter));
  }
    {
      auto stage = std::make_unique<stages::Connect>(
          "move to place", stages::Connect::GroupPlannerVector{ { parameters.arm_group_name_, sampling_planner } });
      stage->setTimeout(5.0);
      stage->properties().configureInitFrom(Stage::PARENT);
      addStageToTask(std::move(stage));
    }
    auto fallbacks = std::make_unique<Fallbacks>("move to other side");
  
    auto place_poses = parameters.place_poses_.find(parameters.support_surfaces_[0])->second;
    geometry_msgs::Pose place_pose; 

    // Get an open place pose and if the object was placed somewhere else then remove it
    for(auto poses : place_poses)
    {
      if(poses.second == "")
      {
        place_pose = place_poses[0].first;
        poses.second = parameters.object_name_;
      }
      else if(poses.second == parameters.object_name_)
      {
        poses.second = "";
      }
    }

    for(auto grasp_frame_transform : parameters.grasp_frame_transforms_)
    {
      auto place = std::make_unique<SerialContainer>("place object");
      exposeTo(*place, { "eef", "hand", "group" });
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

        // Set target pose
        geometry_msgs::PoseStamped p;
        
        p.header.frame_id = parameters.support_surfaces_[0];  // collision_object.header.frame_id;  // object_reference_frame was used before
        p.pose = place_pose; 
        p.pose.orientation.w = 1; 
        p.pose.position.z = getHeightOffsetForSurface(parameters.object_name_, parameters.support_surfaces_[0], parameters.place_surface_offset_);

        stage->setPose(p);
        stage->setMonitoredStage(current_state_stage_);

        // Compute IK
        auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(8);
        wrapper->setIKFrame(grasp_frame_transform.second, parameters.hand_frame_);
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
        stage->allowCollisions(parameters.object_name_, getLinkModelNamesWithCollisionGeometry(parameters.hand_group_name_), false);
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
      fallbacks->insert(std::move(place));
    }
    addStageToTask(std::move(fallbacks));
      /******************************************************
     *                                                    *
     *          Move to Home                              *
     *                                                    *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
      stage->properties().configureInitFrom(Stage::PARENT, { "group" });
      stage->setGoal(parameters.arm_ready_pose_);
      stage->restrictDirection(stages::MoveTo::FORWARD);
      addStageToTask(std::move(stage));
    }

  return initTask();
}
