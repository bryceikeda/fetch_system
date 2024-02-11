#ifndef TASKBASE_H
#define TASKBASE_H

#include <manipulation/manipulation_parameters.h>
#include <moveit/task_constructor/task.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

using namespace moveit::task_constructor; 

class TaskBase
{
public:
  TaskBase(const std::string& task_name);
  virtual ~TaskBase(){};
  virtual bool init(const ManipulationParameters& parameters) = 0;
  bool plan();
  void preempt();
  bool execute();
  void getSolutionMsg(moveit_task_constructor_msgs::Solution& solution);
  void TASK_INFO(const std::string& info);
  void publishSolution(); 
  void setProperty(const std::string& group, const std::string& name);
  void exposeTo(SerialContainer& container, const std::initializer_list<std::string>& properties); 
  const robot_model::JointModelGroup* getJointModelGroup(const std::string& group_name);
  std::vector<std::string> getLinkModelNamesWithCollisionGeometry(const std::string& group_name);
  void addStageToTask(Stage::pointer&& stage);
  bool initTask();

protected:
  Stage* current_state_stage_;

private:
  TaskPtr task_;
  const std::string task_name_;
};

#endif
