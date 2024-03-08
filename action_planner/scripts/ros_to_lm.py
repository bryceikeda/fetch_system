#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json
import requests

class ROStoLMInterface:
    def __init__(self):
        self.post_url = "http://a824-35-229-78-154.ngrok-free.app/post/main/"
        self.action_plan_request = rospy.Subscriber("user_input/speech", String, self.handle_speech_input)
        
    def handle_speech_input(self, msg):
        task = msg.data
        print("[ros_to_lm] Task: " + task)
        msg = {
            "mode": "generate action plan",
            "task": task,
            "object_names": ["pringles", "mustard", "cheezeit"],
            "surface_names": ["table on the right", "table on the left"],
            "action_list": ["pick <object>", "place on <surface>", "done"],
            "feedback": "",
            "action_plan": []
        }

        pub_json = json.dumps(msg)
        requests.post(self.post_url, json=pub_json)

    def run(self):
        rospy.init_node('ROS_to_LLM', anonymous=True)
        rospy.spin()

if __name__ == '__main__':
    ros_lm_interface = ROStoLMInterface()
    rospy.init_node('ROS_to_LLM', anonymous=True)
    rospy.spin()