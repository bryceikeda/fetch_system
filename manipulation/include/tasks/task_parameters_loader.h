#ifndef TASK_PARAMETERS_LOADER_H
#define TASK_PARAMETERS_LOADER_H

#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/RobotState.h>
#include <tasks/task_parameters.h>

class TaskParametersLoader {
    public:
        TaskParametersLoader();
        void loadParameters(const ros::NodeHandle& pnh);
        TaskParameters& getParameters(); 
    private:
        TaskParameters parameters;
        // Which transform to use
        int transform_orientation;
        // .02 or .1665
        std::vector<double> diagonal_frame_transform {0.02, 0, 0.0, 0, 3.9275, 0.0};
        std::vector<double> horizontal_frame_transform {0.02, 0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> vertical_frame_transform {0.02, 0, 0.0,  0, 4.713, 0.0};
};

#endif
