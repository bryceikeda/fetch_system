#include <iostream>
#include "manipulation/manipulation.h"
#include "tasks/task_parameters_loader.h"

constexpr char LOGNAME[] = "pick place test";

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pick_place_test");
  ros::NodeHandle nh, pnh("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Duration(1.0).sleep();

  auto manipulation = std::make_unique<Manipulation>(); 
  auto param_loader = std::make_unique<TaskParametersLoader>(); 
  
  param_loader->loadParameters(pnh);  
  manipulation->setParameters(param_loader->getParameters());

  manipulation->TestPickPlace(); 

  // Keep introspection alive
  //ros::waitForShutdown();
  return 0;
}
