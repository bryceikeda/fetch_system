#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import actionlib
from action_planner.msg import ExecuteActionPlanFeedback, ExecuteActionPlanResult, ExecuteActionPlanAction, ExecuteActionPlanGoal
from manipulation.msg import ManipulationPlanRequest
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveItErrorCodes

class ActionPlannerNode:
    def __init__(self):
        rospy.init_node("action_planner_node")
        self.llm_output_subscriber = rospy.Subscriber('action_planner/llm_output', String, self.handle_llm_output)
        self.action_plan_client = actionlib.SimpleActionClient('execute_action_plan', ExecuteActionPlanAction)
        self.action_plan_client.wait_for_server()

        self.llm_output_msg = String()
        self.llm_output_msg.data = "pick mustard\nplace on table on the left\npick pringles\nplace on table on the left\npick cheezeit\nplace on table on the left\nwave at me"
        
        self.action_plan_goal = ExecuteActionPlanGoal()
        self.action_plan_feedback = ExecuteActionPlanFeedback()
        self.action_plan_result = ExecuteActionPlanResult()

        self.is_executing = False

    def handle_llm_output(self, msg):
        self.llm_output_msg = msg

    def build_action_request(self, task_type, target_object_name="", description="", place_pose=Pose()):
        req = ManipulationPlanRequest()
        req.task_type = task_type
        req.target_object_name = target_object_name
        req.task_name = description
        req.place_pose = place_pose
        return req

    def parse_action_plan(self):
        action_plan = self.llm_output_msg.data.split("\n")

        for task in action_plan:
            self.parse_text(task)

    def parse_text(self, task):
        if task.startswith("pick "):
            self.add_action_request(ManipulationPlanRequest.PICK, task[5:], task)
        elif task.startswith("place on "):
            self.add_action_request(ManipulationPlanRequest.PLACE, task[9:], task)
        elif task.startswith("pour in "):
            self.add_action_request(ManipulationPlanRequest.POUR, task[8:], task)
        elif task == "wave at me":
            self.add_action_request(ManipulationPlanRequest.WAVE, "", task)
        elif task == "dance":
            self.add_action_request(ManipulationPlanRequest.DANCE, "", task)
        else:
            rospy.loginfo("Invalid task: %s", task)

    def add_action_request(self, task_type, target_object_name, description):
        self.action_plan_goal.action_plan.append(self.build_action_request(task_type, target_object_name, description))
        rospy.loginfo(f"{task_type}: {description}")

    def feedback_callback(self, feedback):
        rospy.loginfo("[ActionPlannerNode]: Feedback -> Action executing is %s", feedback.current_action.task_name)

    def execute_action_plan(self):
        self.action_plan_client.send_goal(self.action_plan_goal, feedback_cb=self.feedback_callback)
        print("done")

    def run(self):
        if not self.is_executing and self.llm_output_msg.data:
            self.parse_action_plan()
            self.execute_action_plan()
            self.is_executing = True

    def check_execution(self):
        result = self.get_result()
        
        if self.action_plan_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            if result == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("[ActionPlannerNode]: Action Plan Execution Succeeded")
            elif result == MoveItErrorCodes.PLANNING_FAILED:
                rospy.loginfo("[ActionPlannerNode]: Action Plan Execution Failed")
            elif result == MoveItErrorCodes.CONTROL_FAILED:
                rospy.loginfo("[ActionPlannerNode]: Action Plan Execution Control Failed")
            self.is_executing = False
        elif self.action_plan_client.get_state() == actionlib.GoalStatus.ABORTED:
            rospy.loginfo("[ActionPlannerNode]: Action Plan Execution Aborted")
            self.is_executing = False
        elif self.action_plan_client.get_state() == actionlib.GoalStatus.PREEMPTED:
            rospy.loginfo("[ActionPlannerNode]: Action Plan Execution Preempted")
            self.is_executing = False
    def get_result(self):
        return self.action_plan_client.get_result()

if __name__ == "__main__":
    action_planner_node = ActionPlannerNode()

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        action_planner_node.run()
        action_planner_node.check_execution()
        rate.sleep()