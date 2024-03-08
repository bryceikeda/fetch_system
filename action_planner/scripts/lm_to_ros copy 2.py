#!/usr/bin/env python3

# import requests, json
import rospy
from std_msgs.msg import String
import json
import requests
from flask import Flask, jsonify, request
import threading

app = Flask(__name__)
@app.route("/post/main/", methods=["POST"])
def main_func():
    query = json.loads(request.get_json())
    print(query)
    pub.publish(str('\n'.join(query["action_plan"])))
    return 'true'

if __name__ == "__main__":
    # rospy.init_node('LM_to_ROS', disable_signals=True)
    threading.Thread(target=lambda: rospy.init_node('LM_to_ROS', disable_signals=True)).start()
    pub = rospy.Publisher('action_planner/action_plan', String, queue_size=10)
    # main_func()
    app.run()