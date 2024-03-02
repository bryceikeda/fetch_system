#include <tasks/task_parameters_loader.h>

constexpr char LOGNAME[] = "task_parameters_loader";

TaskParametersLoader::TaskParametersLoader() {}

void TaskParametersLoader::loadParameters(const ros::NodeHandle &pnh_)
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
    
    std::vector<std::string> grasp_frame_transform_names;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "grasp_frame_transform_names", grasp_frame_transform_names);
    
    std::vector<double> grasp_frame_transform_list;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "grasp_frame_transforms", grasp_frame_transform_list);

    for (int i = 0; i < grasp_frame_transform_names.size(); i++)
    {
        Eigen::Isometry3d grasp_frame_transform;
        if (!rosparam_shortcuts::convertDoublesToEigen(grasp_frame_transform_names[i], std::vector<double>(grasp_frame_transform_list.begin() + i*6, grasp_frame_transform_list.begin() + i*6+6), grasp_frame_transform))
        {
            ROS_ERROR_NAMED(LOGNAME, "Failed to convert grasp_frame_transform");
        }
        parameters.grasp_frame_transforms_[grasp_frame_transform_names[i]] = grasp_frame_transform;
    }

    // Predefined pose targets
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_open_pose", parameters.hand_open_pose_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_close_pose", parameters.hand_close_pose_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_ready_pose", parameters.arm_ready_pose_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_tuck_pose", parameters.arm_tuck_pose_);

    // Pick/Place metrics
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_min_dist", parameters.approach_object_min_dist_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_max_dist", parameters.approach_object_max_dist_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_min_dist", parameters.lift_object_min_dist_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_max_dist", parameters.lift_object_max_dist_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "place_surface_offset", parameters.place_surface_offset_);

    // Could be used in the future, not used right now. Included because bottle was being placed sideways
    moveit_msgs::Constraints upright_constraint;
    upright_constraint.name = "upright_constraint";
    upright_constraint.orientation_constraints.resize(1);
    {
        moveit_msgs::OrientationConstraint &c = upright_constraint.orientation_constraints[0];
        c.link_name = parameters.hand_frame_;       // constraining hand frame
        c.header.frame_id = parameters.base_frame_; // reference the base frame
        c.orientation.w = 1.0;
        c.absolute_y_axis_tolerance = 0.65;
        c.absolute_z_axis_tolerance = 0.65;
        c.absolute_x_axis_tolerance = M_PI;
        c.weight = 1.0;
    }
    parameters.constraints_[upright_constraint.name] = upright_constraint;

    moveit_msgs::Constraints downward_constraint;
    upright_constraint.name = "downward_constraint";
    upright_constraint.orientation_constraints.resize(1);
    {
        moveit_msgs::OrientationConstraint &c = upright_constraint.orientation_constraints[0];
        c.link_name = parameters.hand_frame_;       // constraining hand frame
        c.header.frame_id = parameters.base_frame_; // reference the base frame
        c.orientation.w = -1.0;
        c.absolute_y_axis_tolerance = 0.65;
        c.absolute_z_axis_tolerance = 0.65;
        c.absolute_x_axis_tolerance = M_PI;
        c.weight = 1.0;
    }
    parameters.constraints_[downward_constraint.name] = downward_constraint;

    rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

TaskParameters &
TaskParametersLoader::getParameters()
{
    return parameters;
}
