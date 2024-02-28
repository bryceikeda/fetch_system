#!/usr/bin/env python3

import rospy
from manipulation.srv import GetManipulationPlan  # Import correct service type
from manipulation.msg import ManipulationPlanRequest
from actionlib.simple_action_client import SimpleActionClient
from moveit_task_constructor_msgs.msg import ExecuteTaskSolutionAction, ExecuteTaskSolutionGoal
from geometry_msgs.msg import Pose, Point, Quaternion  # Import Pose, Point, and Quaternion
from std_msgs.msg import Duration
from moveit_msgs.msg import MoveItErrorCodes


#  ManipulationPlanRequest.PICK
#  ManipulationPlanRequest.PLACE
#  ManipulationPlanRequest.POUR
#  ManipulationPlanRequest.WIPE
#  ManipulationPlanRequest.OPEN_HAND
#  ManipulationPlanRequest.CLOSE_HAND
#  ManipulationPlanRequest.WAVE
#  ManipulationPlanRequest.DANCE

def build_action_request(task_type, object_name = "", support_surface = "", description = "", place_pose = Pose()):
    req = ManipulationPlanRequest()
    req.task_type = task_type
    req.object_name = object_name
    req.support_surfaces = support_surface
    req.task_name = description
    req.place_pose = place_pose
    return req

def main():
    rospy.init_node("executive_node")
    # Use a context manager for the spinner
    manipulation_plan_service = rospy.ServiceProxy("get_manipulation_plan", GetManipulationPlan)  # Fix service type
    
    # Use a more descriptive variable name for the action client
    plan_executer_client = SimpleActionClient("execute_task_solution", ExecuteTaskSolutionAction)
    plan_executer_client.wait_for_server()

    action_plan = [ 
        build_action_request(ManipulationPlanRequest.PICK, "sponge", "table1", "Pick sponge"),
        build_action_request(ManipulationPlanRequest.PLACE, "sponge", "table1", "Place sponge")
    ]
        
    plan_to_execute = ExecuteTaskSolutionGoal()

    for task in action_plan:
        response = manipulation_plan_service(task)

        if response:
            rospy.loginfo("Plan received, executing plan")
            plan_to_execute.solution = response.manipulation_plan_response.solution
            plan_executer_client.send_goal_and_wait(plan_to_execute)
            
            result = plan_executer_client.get_result().error_code
            
            if result.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Execution Success, moving on to the next task")
            else:
                rospy.logerror("Execution Failed, exiting")
                return 1
        else:
            rospy.logerror("Failed to call service get_manipulation_plan, exiting")
            return 1

if __name__ == "__main__":
    main()
