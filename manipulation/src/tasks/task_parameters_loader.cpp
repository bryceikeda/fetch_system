#include <tasks/task_parameters_loader.h>
 
constexpr char LOGNAME[] = "task_parameters_loader";

TaskParametersLoader::TaskParametersLoader(){}

void
TaskParametersLoader::loadParameters(const ros::NodeHandle& pnh_)
{ 
    /****************************************************
     *                                                  *
     *               Load Parameters                    *
     *                                                  *
     ***************************************************/
    ROS_INFO_NAMED(LOGNAME, "Loading task parameters");
    
    // Planning group properties
    size_t errors = 0;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_group_name", parameters.arm_group_name_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_group_name", parameters.hand_group_name_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "eef_name", parameters.eef_name_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_frame", parameters.hand_frame_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "base_frame", parameters.base_frame_);
    
    // Predefined pose targets
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "gripper_open", parameters.gripper_open_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "gripper_close", parameters.gripper_close_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_ready", parameters.arm_ready_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_tuck", parameters.arm_tuck_);
    
    std::string surface_link;
    // Target object
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "surface_link", surface_link);
    parameters.support_surfaces_ = { surface_link};
    // Pick/Place metrics
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_min_dist", parameters.approach_object_min_dist_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_max_dist", parameters.approach_object_max_dist_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_min_dist", parameters.lift_object_min_dist_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_max_dist", parameters.lift_object_max_dist_  );
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "place_surface_offset", parameters.place_surface_offset_  );
    
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "pick_object", parameters.object_name_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "place_pose", parameters.place_pose_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "object_offset", parameters.object_offset_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "transform_orientation", transform_orientation);
    
    // Should do this better at some point
    if(transform_orientation == 0){
      rosparam_shortcuts::convertDoublesToEigen(LOGNAME, horizontal_frame_transform ,parameters.grasp_frame_transform_);
    }
    else if(transform_orientation == 1){
      rosparam_shortcuts::convertDoublesToEigen(LOGNAME, vertical_frame_transform ,parameters.grasp_frame_transform_);
    }
    else{
      rosparam_shortcuts::convertDoublesToEigen(LOGNAME, diagonal_frame_transform ,parameters.grasp_frame_transform_);
    }
    
    rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

TaskParameters&
TaskParametersLoader::getParameters(){
        return parameters; 
}
