#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import actionlib
from action_planner.msg import (
    ExecuteActionPlanFeedback,
    ExecuteActionPlanResult,
    ExecuteActionPlanAction,
    ExecuteActionPlanGoal,
)
from moveit_msgs.msg import MoveItErrorCodes

from language_model.language_model import LanguageModel
from action_planner.action_plan_parser import ActionPlanParser


class ActionPlannerNode:
    def __init__(self):
        rospy.init_node("action_planner_node")
        self.action_plan_request = rospy.Subscriber(
            "user_input/speech", String, self.handle_speech_input
        )
        self.action_plan_client = actionlib.SimpleActionClient(
            "execute_action_plan", ExecuteActionPlanAction
        )
        self.action_plan_client.wait_for_server()
        rospy.loginfo("[ActionPlannerNode]: Action Plan Client Started")

        self.language_model = LanguageModel()
        self.action_plan_parser = ActionPlanParser()

        self.language_model_output = []
        #self.language_model_output = ["pick pringles", "done"]
        # self.speech_command = (
        #     "pick up the pringles and place it on the table on the left."
        # )
        self.speech_command = ""
        self.goal = ExecuteActionPlanGoal()
        self.feedback = ExecuteActionPlanFeedback()
        self.result = ExecuteActionPlanResult()

        self.is_executing = False

    def handle_speech_input(self, msg):
        self.speech_command = msg.data

    def query_language_model(self):
        query = {
            "mode": "generate action plan",
            "task": self.speech_command,
            "object_names": ["pringles", "mustard", "cheezeit"],
            "surface_names": ["table on the right", "table on the left"],
            "action_list": [
                "pick <object>",
                "place on <surface>",
                "wave at me",
                "done",
                "action plan",
            ],
            "feedback": "",
            "action_plan": [],
        }
        self.language_model_output = self.language_model.query_language_model(query)

    def run(self):
        if not self.is_executing:
            if self.speech_command != "":
                self.query_language_model()
                self.speech_command = ""
            elif self.language_model_output != []:
                self.goal = self.action_plan_parser.get_action_plan_goal(
                    self.language_model_output
                )
                self.language_model_output = []
                self.is_executing = True
                self.execute_action_plan()
        elif self.is_executing:
            self.check_execution()

    def execute_action_plan(self):
        self.action_plan_client.send_goal(
            self.goal, feedback_cb=self.feedback_callback
        )

    def feedback_callback(self, feedback):
        self.feedback = feedback
        rospy.loginfo(
            "[ActionPlannerNode]: Feedback -> Action executing is %s",
            feedback.current_action.task_name,
        )

    def check_execution(self):
        self.action_plan_client.wait_for_result()
        self.result = self.action_plan_client.get_result()
        if self.action_plan_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            if self.result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("[ActionPlannerNode]: Action Plan Execution Succeeded")
            elif self.result.error_code.val == MoveItErrorCodes.PLANNING_FAILED:
                rospy.loginfo("[ActionPlannerNode]: Action Plan Execution Failed")
            elif self.result.error_code.val == MoveItErrorCodes.CONTROL_FAILED:
                rospy.loginfo(
                    "[ActionPlannerNode]: Action Plan Execution Control Failed"
                )
            self.is_executing = False
        elif self.action_plan_client.get_state() == actionlib.GoalStatus.ABORTED:
            rospy.loginfo("[ActionPlannerNode]: Action Plan Execution Aborted")
            self.is_executing = False
        elif self.action_plan_client.get_state() == actionlib.GoalStatus.PREEMPTED:
            rospy.loginfo("[ActionPlannerNode]: Action Plan Execution Preempted")
            self.is_executing = False

if __name__ == "__main__":
    action_planner_node = ActionPlannerNode()
    rospy.sleep(3)

    rate = rospy.Rate(10)  # 10hz
    # action_planner_node.query_language_model()
    while not rospy.is_shutdown():
        action_planner_node.run()
        rate.sleep()
