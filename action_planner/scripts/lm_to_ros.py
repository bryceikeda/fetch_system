#!/usr/bin/env python3

import rospy
import json
from flask import Flask, jsonify, request
import threading
from std_msgs.msg import String

class LMtoROSInterface:
    def __init__(self):
        self.app = Flask(__name__)
        self.app.route("/post/main/", methods=["POST"])(self.query_callback)
        self.query = None
        self.llm_output_publisher = rospy.Publisher('action_planner/llm_output', String, queue_size=10)

    def query_callback(self):
        response = json.loads(request.get_json())
        self.query = str('\n'.join(response["action_plan"]))
        self.llm_output_publisher.publish(self.query)

    def run(self):
        threading.Thread(target=lambda: rospy.init_node('LM_to_ROS', disable_signals=True)).start()
        self.app.run()

if __name__ == "__main__":
    lm_ros_interface = LMtoROSInterface()
    lm_ros_interface.run()