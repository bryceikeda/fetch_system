#include <tasks/pour_task.h>

  const bool registered = TaskFactory::registerTask(
      manipulation::ManipulationPlanRequest::POUR,
      [](const std::string& taskName, const ros::NodeHandle& nh) -> std::unique_ptr<TaskBase> {
          return std::make_unique<PourTask>(taskName, nh);
      }
  );

using namespace moveit::task_constructor; 

PourTask::PourTask(const std::string& task_name, const ros::NodeHandle& nh) : TaskBase(task_name, nh)
{
  current_state_stage_ = nullptr;
  attach_object_stage_ = nullptr;
}

bool PourTask::init(const TaskParameters& parameters)
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
  
          if (s.start()->scene()->getCurrentState().hasAttachedBody(parameters.object_name_))
          {
            comment = "object with id '" + parameters.object_name_ + "' is already attached and cannot be picked";
            return false;
          }
        return true;
      });
  
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
  
      // Move to pick stage
      {
        auto stage = std::make_unique<stages::Connect>(
            "move to pick", stages::Connect::GroupPlannerVector{ { parameters.arm_group_name_, sampling_planner } });
        stage->setTimeout(5.0);
        stage->properties().configureInitFrom(Stage::PARENT);
        addStageToTask(std::move(stage));
      }
  
      // approach object
      {
        auto grasp = std::make_unique<SerialContainer>("pick object");
        exposeTo(*grasp, { "eef", "hand", "group", "ik_frame" });
        grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });
  
        /****************************************************
  
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
          // parameters.grasp_frame_transform_
          // Raise it up -.1
          wrapper->setIKFrame(Eigen::Translation3d(0.0, 0, -0.1), parameters.hand_frame_);
          wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
          wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
          grasp->insert(std::move(wrapper));
        }
  
        /****************************************************
      ---- *               Allow Collision (hand object)   *
         ***************************************************/
        {
          auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
          stage->allowCollisions(parameters.object_name_, getLinkModelNamesWithCollisionGeometry(parameters.hand_group_name_),
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
  
        addStageToTask(std::move(grasp));
      }
    }
    /***************************************************/
    /****************************************************
      .... *            PICK END   **************** *
    ***************************************************/
    /***************************************************/
    /***************************************************/
    moveit_msgs::Constraints upright_constraint;
	upright_constraint.name = "pouring_hand_constraints";
	upright_constraint.orientation_constraints.resize(1);
	{
		moveit_msgs::OrientationConstraint& c= upright_constraint.orientation_constraints[0];
		c.link_name= parameters.hand_frame_;   //constraining hand frame
		c.header.frame_id= parameters.base_frame_; //reference the base frame
		c.orientation.w= 1.0;

		c.absolute_y_axis_tolerance= 0.65;
		c.absolute_z_axis_tolerance= 0.65;
		c.absolute_x_axis_tolerance= M_PI;

		c.weight= 1.0;
	}
   {
    auto stage = std::make_unique<stages::Connect>(
        "move to pre-pour pose",
        stages::Connect::GroupPlannerVector{{parameters.arm_group_name_, sampling_planner}});
    stage->setTimeout(15.0);
    stage->setPathConstraints(upright_constraint);
    stage->properties().configureInitFrom(Stage::PARENT); // TODO: convenience-wrapper
    addStageToTask(std::move(stage));
   }

   {
    auto stage = std::make_unique<stages::GeneratePose>("pose above glass");
    stage->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame"});//
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "glass";
    p.pose.orientation.x = 0.0;
    p.pose.orientation.z = 0.0;
    p.pose.orientation.w = 1.0;
    p.pose.position.z = .4;
    stage->setPose(p);
    stage->properties().configureInitFrom(Stage::PARENT);

    stage->setMonitoredStage(attach_object_stage_);

    auto wrapper = std::make_unique<stages::ComputeIK>("pre-pour pose", std::move(stage));
    wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame"});//
    wrapper->setMaxIKSolutions(8);
    wrapper->setIKFrame(parameters.grasp_frame_transforms_.find("horizontal_grasp_transform")->second, parameters.hand_frame_);
    // TODO adding this will initialize "target_pose" which is internal (or
    // isn't it?)
    // wrapper->properties().configureInitFrom(Stage::PARENT);
    wrapper->properties().property("eef").configureInitFrom(Stage::PARENT, "eef");//
    wrapper->properties().property("group").configureInitFrom(Stage::PARENT, "group");//
    wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
    addStageToTask(std::move(wrapper));
  }

  {
    auto stage = std::make_unique<manipulation_stages::PourInto>("pouring");
    stage->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame"});//
    stage->properties().property("group").configureInitFrom(Stage::PARENT, "group");//
    stage->setBottle(parameters.object_name_);
    stage->setContainer("glass");
    stage->setPourOffset(Eigen::Vector3d(0, 0.02, 0.01));
    stage->setTiltAngle(2.0);
    stage->setPourDuration(ros::Duration(4.0));
    {
      geometry_msgs::Vector3Stamped pouring_axis;
      pouring_axis.header.frame_id = "glass";
      pouring_axis.vector.x = 1.0;
      stage->setPouringAxis(pouring_axis);
    }
    stage->properties().configureInitFrom(Stage::PARENT);
    addStageToTask(std::move(stage));
  }

 /******************************************************
      ---- *          Generate Place Pose                       *
         *****************************************************/
     { 
        auto stage = std::make_unique<stages::Connect>(
            "move to place", stages::Connect::GroupPlannerVector{ { parameters.arm_group_name_, sampling_planner } });
        stage->setTimeout(5.0);
        stage->setPathConstraints(upright_constraint);
        stage->properties().configureInitFrom(Stage::PARENT);
        addStageToTask(std::move(stage));
      }
  
      {
        auto place = std::make_unique<SerialContainer>("place object");
        exposeTo(*place, { "eef", "hand", "group" });
        place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group" });
  
        {
          // Generate Place Pose
          auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
          stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
          stage->properties().set("marker_ns", "place_pose");
          stage->setObject(parameters.object_name_);
  
          // Set target pose
          geometry_msgs::PoseStamped p;
          p.header.frame_id = parameters.base_frame_;  // collision_object.header.frame_id;  // object_reference_frame was used before
          p.pose = parameters.place_pose_;
          p.pose.position.z = .7;  
          stage->setPose(p);
          stage->setMonitoredStage(attach_object_stage_);
  
          // Compute IK
          auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
          wrapper->setMaxIKSolutions(8);
          wrapper->setIKFrame(parameters.grasp_frame_transforms_.find("horizontal_grasp_transform")->second, parameters.hand_frame_);
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
          addStageToTask(std::move(place));
        }
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
