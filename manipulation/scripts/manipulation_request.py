#!/usr/bin/env python3

import rospy
from manipulation.srv import GetManipulationPlan  # Import correct service type
from manipulation.msg import ManipulationPlanRequest
from actionlib.simple_action_client import SimpleActionClient
from moveit_task_constructor_msgs.msg import ExecuteTaskSolutionAction, ExecuteTaskSolutionGoal
from geometry_msgs.msg import Pose, Point, Quaternion  # Import Pose, Point, and Quaternion
from std_msgs.msg import Duration
from moveit_msgs.msg import MoveItErrorCodes

LOGNAME = "manipulation_request"

def main():
    rospy.init_node("manipulation_request")
    # Use a context manager for the spinner
    get_plan = rospy.ServiceProxy("get_manipulation_plan", GetManipulationPlan)  # Fix service type
    manipulation_request = ManipulationPlanRequest()
    manipulation_request.task_type = ManipulationPlanRequest.WIPE
    manipulation_request.object_name = "sponge"
    manipulation_request.support_surfaces = ["table1"]
    manipulation_request.task_name = "task_request"

    # Use dictionary-style initialization for Pose
    manipulation_request.place_pose = Pose(
        position=Point(0.8, 0.11, 0.8),
        orientation=Quaternion(0, 0, 0, 1.0)
    )

    response = get_plan(manipulation_request)

    # Use a more descriptive variable name for the action client
    execute_client = SimpleActionClient("execute_task_solution", ExecuteTaskSolutionAction)
    execute_client.wait_for_server()

    execute_goal = ExecuteTaskSolutionGoal()

    if response:
        rospy.loginfo("Plan received, executing plan")
        execute_goal.solution = response.manipulation_plan_response.solution
        execute_client.send_goal_and_wait(execute_goal)
        execute_result = execute_client.get_result().error_code
        if execute_result.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Execution Success")
            return 0
        else:
            rospy.loginfo("Execution Failed")
            return 1
    else:
        rospy.logerror("Failed to call service get_manipulation_plan")
        return 1



if __name__ == "__main__":
    main()
