#!/usr/bin/env python3

import rospy
from manipulation.srv import GetManipulationPlan  # Import correct service type
from manipulation.msg import ManipulationPlanRequest
from actionlib.simple_action_client import SimpleActionClient
from moveit_task_constructor_msgs.msg import (
    ExecuteTaskSolutionAction,
    ExecuteTaskSolutionGoal,
)
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)  # Import Pose, Point, and Quaternion
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


def build_action_request(
    task_type, object_name="", target_name="", description="", place_pose=Pose()
):
    req = ManipulationPlanRequest()
    req.task_type = task_type
    req.object_name = object_name
    req.target_name = target_name
    req.task_name = description
    req.place_pose = place_pose
    return req


def main():
    rospy.init_node("executive_node")
    # Use a context manager for the spinner
    manipulation_plan_service = rospy.ServiceProxy(
        "get_manipulation_plan", GetManipulationPlan
    )  # Fix service type

    # Use a more descriptive variable name for the action client
    plan_executer_client = SimpleActionClient(
        "execute_task_solution", ExecuteTaskSolutionAction
    )

    pour_plan = [
        build_action_request(ManipulationPlanRequest.PICK, "bottle", "", "Pick bottle"),
        # #build_action_request(ManipulationPlanRequest.WIPE, "sponge", "table1", "Wipe table"),
        build_action_request(
            ManipulationPlanRequest.POUR, "bottle", "glass", "Pour bottle"
        ),
        build_action_request(ManipulationPlanRequest.PLACE, "bottle", "table1", "Place bottle")
    ]

    wipe_plan = [
        build_action_request(ManipulationPlanRequest.PICK, "sponge", "", "Pick sponge"),
        # #build_action_request(ManipulationPlanRequest.WIPE, "sponge", "table1", "Wipe table"),
        build_action_request(
            ManipulationPlanRequest.WIPE, "sponge", "table1", "Pour bottle"
        ),
        build_action_request(ManipulationPlanRequest.PLACE, "sponge", "table1", "Place sponge")
    ]

    picking_sequence = [
        build_action_request(ManipulationPlanRequest.PICK, "meat can", "", "Pick meat can"),
        build_action_request(ManipulationPlanRequest.PLACE, "meat can", "table1", "Place mean can")
    ]


    plan_to_execute = ExecuteTaskSolutionGoal()

    max_tries = 2

    for task in picking_sequence:
        current_tries = 0
        while current_tries < max_tries:
            response = manipulation_plan_service(task)
            if response.manipulation_plan_response.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Plan received, executing plan")
                current_tries = 0
                break 
            else:
                rospy.loginfo("Failed to get plan, trying again")
                current_tries += 1

        if current_tries == max_tries:
            rospy.logerror("Failed to get plan, exiting")
            return 1
        else:
            plan_to_execute.solution = response.manipulation_plan_response.solution
            plan_executer_client.send_goal_and_wait(plan_to_execute)

            result = plan_executer_client.get_result().error_code

            if result.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Execution Success, moving on to the next task")
            else:
                rospy.logerror("Execution Failed, exiting")
                return 1

if __name__ == "__main__":
    main()
