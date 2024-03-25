#!/usr/bin/env python3

import rospy
from action_planner.msg import ExecuteActionPlanGoal
from manipulation.msg import ManipulationPlanRequest
from geometry_msgs.msg import Pose


class ActionPlanParser:
    def __init__(self):
        self.picked_object = ""
        self.action_plan_goal = ExecuteActionPlanGoal()
        self.plan_id = 0

    def get_action_plan_goal(self, language_model_output):
        self.action_plan_goal = ExecuteActionPlanGoal()
        for task in language_model_output:
            self.parse_text(task)
        return self.action_plan_goal

    def parse_text(self, task):
        if task.startswith("pick "):
            self.picked_object = task[5:]
            self.add_action_request(ManipulationPlanRequest.PICK, task[5:], "Action " + str(self.plan_id) + ": " + task)
        elif task.startswith("place on "):
            self.add_action_request(
                ManipulationPlanRequest.PLACE, task[9:], "Action " + str(self.plan_id) + ": " + self.picked_object + " " + task
            )
        elif task.startswith("pour in "):
            self.add_action_request(ManipulationPlanRequest.POUR, task[8:], "Action " + str(self.plan_id) + ": " + task)
        elif task == "wave at me":
            self.add_action_request(ManipulationPlanRequest.WAVE, "", "Action " + str(self.plan_id) + ": " + task)
        elif task == "dance":
            self.add_action_request(ManipulationPlanRequest.DANCE, "", "Action " + str(self.plan_id) + ": " + task)
        elif task == "done":
            self.add_action_request(ManipulationPlanRequest.DONE, "", "Action Plan Completed")
        else:
            rospy.loginfo("Invalid task: %s", task)
        self.plan_id += 1

    def build_action_request(
        self, task_type, target_object_name="", description="", place_pose=Pose()
    ):
        req = ManipulationPlanRequest()
        req.task_type = task_type
        req.target_object_name = target_object_name
        req.task_name = description
        req.place_pose = place_pose
        return req

    def add_action_request(self, task_type, target_object_name, description):
        self.action_plan_goal.action_plan.append(
            self.build_action_request(task_type, target_object_name, description)
        )
        rospy.loginfo(f"{task_type}: {description}")
