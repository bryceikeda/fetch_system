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
    Pose
    )  # Import Pose, Point, and Quaternion
from moveit_msgs.msg import MoveItErrorCodes
import actionlib
from action_planner.msg import (
    ExecuteActionPlanFeedback,
    ExecuteActionPlanResult,
    ExecuteActionPlanAction,
    ExecuteActionPlanGoal,
)
from std_srvs.srv import Trigger, TriggerResponse

def build_action_request(task_type, target_object_name="", description="", place_pose=Pose()):
    req = ManipulationPlanRequest()
    req.task_type = task_type
    req.target_object_name = target_object_name
    req.task_name = description
    req.place_pose = place_pose
    return req

class ExecutiveNode:
    def __init__(self):
        rospy.init_node("executive_node")

        self.manipulation_plan_client = rospy.ServiceProxy("get_manipulation_plan", GetManipulationPlan)
        self.plan_executer_client = SimpleActionClient("execute_task_solution", ExecuteTaskSolutionAction)
        self.stop_robot_server = rospy.Service('/executive/stop_robot', Trigger, self.handle_stop_robot_request)
        self.action_plan_server = actionlib.SimpleActionServer("execute_action_plan", ExecuteActionPlanAction, execute_cb=self.execute_action_plan_callback, auto_start=False)
        self.action_plan_server.start()

        
        self.task_solution = ExecuteTaskSolutionGoal()
        self.feedback = ExecuteActionPlanFeedback()
        self.result = ExecuteActionPlanResult()

    def execute_trajectory_action(self, goal):
        print("Executing trajectory action")
        self.action_plan_server.set_succeeded()

    def handle_stop_robot_request(self, req):
        self.plan_executer_client.cancel_goal()
        return TriggerResponse(True, "Robot Stopped")
        
    def handle_action_plan(self, msg):
        self.received_action_plan = msg

    def compute_task_solution(self, task, max_tries=2):
        for _ in range(max_tries):
            response = self.manipulation_plan_client(task)
            if response.manipulation_plan_response.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("[ExecutiveNode]: Plan computed")
                self.task_solution.solution = response.manipulation_plan_response.solution
                return MoveItErrorCodes.SUCCESS
            rospy.loginfo("[ExecutiveNode]: Failed to get plan, trying again...")

        rospy.loginfo("[ExecutiveNode]: Failed to get plan after {} tries, exiting".format(max_tries))
        return MoveItErrorCodes.FAILURE

    def execute_solution(self):
        self.plan_executer_client.send_goal_and_wait(self.task_solution)
        return self.plan_executer_client.get_result().error_code.val

    def execute_action_plan_callback(self, goal):
        status = MoveItErrorCodes.SUCCESS
        for task in goal.action_plan:
            if self.action_plan_server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self.action_plan_server.set_preempted()
                status = MoveItErrorCodes.PREEMPTED
                break

            self.feedback.current_action = task
            self.action_plan_server.publish_feedback(self.feedback)

            if self.compute_task_solution(task) == MoveItErrorCodes.FAILURE:
                rospy.loginfo("[ExecutiveNode]: Failed to get plan, exiting")
                status = MoveItErrorCodes.PLANNING_FAILED
                break
            
            error_code = self.execute_solution()

            if error_code == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("[ExecutiveNode]: Execution Success, moving on to the next task")
            else:
                status = MoveItErrorCodes.CONTROL_FAILED
                rospy.loginfo("[ExecutiveNode]: Execution Failed, exiting")
                break

        self.result.error_code.val = status
        self.action_plan_server.set_succeeded(self.result)

if __name__ == "__main__":
    executive_node = ExecutiveNode()

    rospy.sleep(3)

    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()