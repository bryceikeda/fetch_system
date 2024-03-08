#!/usr/bin/env python3

# import requests, json
import rospy
from std_msgs.msg import String
import json
import requests

post_url = "http://0a4a-34-90-170-107.ngrok-free.app/post/main/"
task_in_progress = False

def action_callback(data):
    global task_in_progress
    task_in_progress = False

def task_callback(data):
    global task_in_progress

    if not task_in_progress:
        task_in_progress = True
        
        task = data.data
        print("[ros_to_lm] Task: " + task)
        query = {"mode": "generate action plan",
                "task": task,
                "object_names": ["Cup", "Pringles", "Mustard", "Bottle", "Cheezeit"],
                "surface_names": ["Table"],
                "action_list": ["pick <object>", "place on <surface>", "pour in <object>", "wave", "done"],
                "feedback": "",
                "action_plan": []}

        pub_json = json.dumps(query)
        r = requests.post(post_url, json=pub_json)

def run_study():
    rospy.init_node('ROS_to_LLM', anonymous=True)
    rospy.Subscriber("user_input/speech", String, task_callback)
    rospy.Subscriber("action_planner/action_plan", String, action_callback)
    rospy.spin()

if __name__ == '__main__':
    run_study()