#include <tasks/pour_task.h>

  const bool registered = TaskFactory::registerTask(
      manipulation::ManipulationPlanRequest::POUR,
      [](const std::string& taskName) -> std::unique_ptr<TaskBase> {
          return std::make_unique<PourTask>(taskName);
      }
  );

using namespace moveit::task_constructor; 

PourTask::PourTask(const std::string& task_name) : TaskBase(task_name)
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
   // don't spill liquid
    moveit_msgs::Constraints upright_constraint;
    upright_constraint.name = parameters.hand_frame_;
    upright_constraint.orientation_constraints.resize(1);
    {
        moveit_msgs::OrientationConstraint &c =
            upright_constraint.orientation_constraints[0];
        c.link_name = parameters.hand_frame_;
        c.header.frame_id = "table1";
        c.orientation.w = 1.0;
        c.absolute_x_axis_tolerance = 0.65;
        c.absolute_y_axis_tolerance = 0.65;
        c.absolute_z_axis_tolerance = M_PI;
        c.weight = 1.0;
    }
      
      
      
     {
    auto _current_state = std::make_unique<stages::CurrentState>("current state");
    _current_state->setTimeout(10);

    // Verify that object is not attached for pouring and if object is attached for placing
    auto applicability_filter =
        std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
    applicability_filter->setPredicate([&](const SolutionBase& s, std::string& comment) {
      s.start()->scene()->printKnownObjects(std::cout);

        if (s.start()->scene()->getCurrentState().hasAttachedBody(parameters.object_name_))
        {
          comment = "object with id '" + parameters.object_name_ + "' is already attached and cannot be poured";
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
      stage->setGoal(parameters.gripper_open_);
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
        vec.header.frame_id = parameters.hand_frame_;
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
        stage->setPreGraspPose(parameters.gripper_open_);
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
        stage->setGoal(parameters.gripper_close_);
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
    /****************************************************
    .... *           Pose above glass -> pre-pour pose  *
    ***************************************************/
    {
        auto stage = std::make_unique<stages::GeneratePose>("pose above glass");
        stage->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame"});
        geometry_msgs::PoseStamped p;
        p.header.frame_id= "glass";
        p.pose.position.z = .2;
        p.pose.orientation.w = 1;
       
        stage->setPose(p);
		stage->properties().configureInitFrom(Stage::PARENT);
		stage->setMonitoredStage(attach_object_stage_);
		
        auto wrapper = std::make_unique<stages::ComputeIK>("pre-pour pose2", std::move(stage) );
		wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame"});//
		wrapper->setMaxIKSolutions(8);
		wrapper->setIKFrame(parameters.grasp_frame_transform_, parameters.hand_frame_);//															// <=================== !!! set ik for transform hand_frame
		wrapper->properties().property("eef").configureInitFrom(Stage::PARENT, "eef");//
		wrapper->properties().property("group").configureInitFrom(Stage::PARENT, "group");//
		wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
		addStageToTask(std::move(wrapper));
        stage->setPose(p);
        stage->properties().configureInitFrom(Stage::PARENT);
        stage->setMonitoredStage(attach_object_stage_);
    }
    /****************************************************
    .... *              Pouring                         *
    ***************************************************/
    {
       auto stage = std::make_unique<manipulation_stages::PourInto>("pouring");
        stage->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame"});
        stage->setBottle("cylinder");
        stage->setContainer("glass");

        stage->setPourOffset(Eigen::Vector3d(0,0.03,0.01));

        // pour_tilt_angle, pour_duration, pour_waypoint_duration
        stage->setTiltAngle(2.0);
        stage->setPourDuration(ros::Duration(4.0)); //changed duration here!! <===
        stage->setWaypointDuration(ros::Duration(.25));
        {
          geometry_msgs::Vector3Stamped pouring_axis;
          pouring_axis.header.frame_id= "glass"; //s_model_tool0
          pouring_axis.vector.x = 1.0; //x //<===== -1
          stage->setPouringAxis(pouring_axis);
        }
        addStageToTask(std::move(stage));
    }
  }

  return initTask();
}
