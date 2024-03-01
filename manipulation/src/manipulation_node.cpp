#include <iostream>
#include "manipulation/manipulation.h"
#include "tasks/task_parameters_loader.h"

constexpr char LOGNAME[] = "manipulation node";

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "manipulation_node");
    ros::NodeHandle nh, pnh("~");

    auto manipulation = std::make_unique<Manipulation>(nh);
    auto param_loader = std::make_unique<TaskParametersLoader>();

    param_loader->loadParameters(pnh);
    manipulation->setParameters(param_loader->getParameters());

    manipulation->get_manipulation_plan_service = nh.advertiseService("get_manipulation_plan", &Manipulation::handleManipulationPlanRequest, manipulation.get());
    manipulation->update_planning_scene_service = nh.serviceClient<std_srvs::Trigger>("update_planning_scene");
    // manipulation->update_planning_scene_service.waitForExistence();

    ros::Duration(5.0).sleep();
    ros::Rate loop_rate(40);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Keep introspection alive
    ros::waitForShutdown();
    return 0;
}
