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
    Stage* attach_object_stage_; 
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
          stage->setPreGraspPose(parameters.hand_open_pose_);
          stage->setObject(parameters.object_name_);
          stage->setAngleDelta(M_PI / 12);
          stage->setMonitoredStage(current_state_stage_);  // Hook into current state
  
          // Compute IK
          auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
          wrapper->setMaxIKSolutions(8);
          wrapper->setMinSolutionDistance(1.0);
          wrapper->setIKFrame(parameters.grasp_frame_transforms_.find("vertical_grasp_transform")->second, parameters.hand_frame_);
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
    // approach object
    {
      auto wipe_table = std::make_unique<SerialContainer>("Move to Wipe");
      exposeTo(*wipe_table, { "eef", "hand", "group", "ik_frame" });
      wipe_table->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });
  
      /****************************************************
      ---- *               Allow Collision (hand object)   *
      ***************************************************/
      {
        auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,surface)");
        stage->allowCollisions(parameters.object_name_, parameters.support_surfaces_, true);
        attach_object_stage_ = stage.get();
        wipe_table->insert(std::move(stage));
      }
      {
        auto stage = std::make_unique<stages::MoveTo>("move to wipe start", sampling_planner);
        stage->setTimeout(5.0);
        stage->setIKFrame(parameters.grasp_frame_transforms_.find("vertical_frame_transform")->second, parameters.hand_frame_);
        stage->properties().configureInitFrom(Stage::PARENT);
        
        geometry_msgs::PoseStamped place_pose;
        place_pose.header.frame_id = parameters.support_surfaces_[0];
        
        // If place pose is set, use it, if not, generate one
        if(parameters.place_pose_.position.z != 0.0)
        {
          place_pose.pose = parameters.place_pose_;
          place_pose.pose.position.z += parameters.place_surface_offset_;
          stage->setGoal(place_pose);
        }
        else
        {
          place_pose.pose.position.z = getHeightOffsetForSurface(parameters.object_name_, parameters.support_surfaces_[0], parameters.place_surface_offset_);
          stage->setGoal(place_pose);
        }
        wipe_table->insert(std::move(stage));
      }
      {
        wipe_table->insert(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "right")));
        wipe_table->insert(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "right")));
        wipe_table->insert(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "right")));
        wipe_table->insert(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "left")));
        wipe_table->insert(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "left")));
        wipe_table->insert(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "left")));
        wipe_table->insert(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "left")));
        wipe_table->insert(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "left")));
        wipe_table->insert(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "left")));
        wipe_table->insert(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "right")));
        wipe_table->insert(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "right")));
        wipe_table->insert(std::move(moveDiagonal(parameters.arm_group_name_, parameters, "right")));
      }


      // {
      //     auto stage = std::make_unique<stages::GeneratePose>("pose on table");
      //     geometry_msgs::PoseStamped p;
      //     p.header.frame_id = "table1";
      //     p.pose.position.z = .79;
      //     p.pose.position.x = -.3;
      //     stage->setPose(p);
      //     stage->properties().configureInitFrom(Stage::PARENT);
          
      //     stage->setMonitoredStage(attach_object_stage_);  // Hook into current state

      //     auto wrapper = std::make_unique<stages::ComputeIK>("on table pose", std::move(stage));
      //     wrapper->setMaxIKSolutions(8);
      //     wrapper->setIKFrame(parameters.grasp_frame_transform_, parameters.hand_frame_);
      //     wrapper->properties().configureInitFrom(Stage::PARENT, {"eef"}); // TODO: convenience wrapper
      //     wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
      //     wipe_table->insert(std::move(wrapper));
      // }
      addStageToTask(std::move(wipe_table));
    }
    return initTask();
}


std::unique_ptr<SerialContainer> WipeTask::moveDiagonal(const std::string& group, const TaskParameters& parameters, const std::string& direction) {
	auto c = std::make_unique<SerialContainer>("Diagonal Path " + direction);
	c->setProperty("group", group);

	// create Cartesian interpolation "planner" to be used in stages
	auto cartesian = std::make_shared<solvers::CartesianPath>();
	// create joint interpolation "planner"
	auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

  double up = .05; 
  double down = -.05;
  double dir = -.05; 

  if (direction == "left") {
    dir = .05;
  }

	{
		auto stage = std::make_unique<stages::MoveRelative>("diagonal upward", cartesian);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		geometry_msgs::Vector3Stamped direction_vec;
		direction_vec.header.frame_id = parameters.base_frame_;
		direction_vec.vector.x = up;
    direction_vec.vector.y = dir;
		stage->setDirection(direction_vec);
		c->insert(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("diagonal downward", cartesian);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		geometry_msgs::Vector3Stamped direction_vec;
		direction_vec.header.frame_id = parameters.base_frame_;
		direction_vec.vector.x = down;
    direction_vec.vector.y = dir;
		stage->setDirection(direction_vec);
		c->insert(std::move(stage));
	}

	return c;
}