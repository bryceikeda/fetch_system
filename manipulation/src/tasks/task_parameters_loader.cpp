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

    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "pick_object", parameters.object_name_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "place_pose", parameters.place_pose_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "object_offset", parameters.object_offset_);

    // Get the different grasp orientations for the robot
    Eigen::Isometry3d horizontal_grasp_frame_transform;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "horizontal_grasp_frame_transform", horizontal_grasp_frame_transform);
    parameters.grasp_frame_transforms_["horizontal_grasp_frame_transform"] = horizontal_grasp_frame_transform;
    Eigen::Isometry3d vertical_grasp_frame_transform;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "vertical_grasp_frame_transform", vertical_grasp_frame_transform);
    parameters.grasp_frame_transforms_["vertical_grasp_frame_transform"] = vertical_grasp_frame_transform;
    Eigen::Isometry3d diagonal_grasp_frame_transform;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "diagonal_grasp_frame_transform", diagonal_grasp_frame_transform);
    parameters.grasp_frame_transforms_["diagonal_grasp_frame_transform"] = diagonal_grasp_frame_transform;

    rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

TaskParameters &
TaskParametersLoader::getParameters()
{
    return parameters;
}
