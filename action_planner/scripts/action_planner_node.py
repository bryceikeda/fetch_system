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
from scene_graph.srv import QuerySceneGraph, QuerySceneGraphRequest, QuerySceneGraphResponse

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

        self.scene_graph_query_client = rospy.ServiceProxy(
            "/scene_graph/query", QuerySceneGraph
        )
        self.scene_graph_query_client.wait_for_service()

        rospy.loginfo("[ActionPlannerNode]: Action Plan Client Started")

        self.language_model = LanguageModel()
        self.action_plan_parser = ActionPlanParser()

        
        self.language_model_output = []
        self.speech_command = ""

        self.language_model_output = ["pick Pringles", "place on table on the right", "pick Cheezeit", "place on table on the right", "pick Cup", "place on table on the right", "done"]
        #self.language_model_output = ["pick Pringles", "place on table on the left", "pick Cheezeit", "place on table on the left", "pick Cup", "place on table on the left", "done"]
        #self.language_model_output = ["wave at me"]
        #self.speech_command = "move the pringles to the table on the left"
        self.goal = ExecuteActionPlanGoal()
        self.feedback = ExecuteActionPlanFeedback()
        self.result = ExecuteActionPlanResult()
        self.object_names = []
        self.surface_names = []
        self.is_executing = False


    def handle_model_states(self, msg):
        self.model_states = msg

    def handle_speech_input(self, msg):
        self.speech_command = msg.data

    def query_scene_graph(self, attribute_name):
        # Construct the query with the provided attribute name
        query = QuerySceneGraphRequest("", "", attribute_name)

        # Call the scene graph query service with the constructed query
        res = self.scene_graph_query_client.call(query)
        print(res.related_nodes)
        # Return the related nodes from the response
        return res.related_nodes
    
    def query_language_model(self):
        self.surface_names = self.query_scene_graph("surface")
        self.object_names = self.query_scene_graph("object")
        
        query = {
            "mode": "generate action plan",
            "task": self.speech_command,
            "object_names": self.object_names,
            "surface_names": self.surface_names,
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
                self.goal = ExecuteActionPlanGoal()
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
            "[ActionPlannerNode]: Feedback -> Executing is %s",
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
            self.speech_command = "wave at me"
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
