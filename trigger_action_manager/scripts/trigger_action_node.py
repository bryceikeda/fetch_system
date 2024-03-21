#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


class TriggerActionNode:
    def __init__(self):
        rospy.init_node("trigger_action_node")
        self.triggered_action_publisher = rospy.Publisher(
            "trigger_action/triggered_action", String, queue_size=10
        )
        self.spawn_trigger_publisher = rospy.Publisher(
            "trigger_action/add_trigger", String, queue_size=10
        )
        self.delete_trigger_subscriber = rospy.Subscriber(
            "unity/delete_trigger", String, self.handle_delete_trigger
        )

        self.trigger_names = []

    def handle_delete_trigger(self, msg):
        if msg.data in self.trigger_names:
            self.trigger_names.remove(msg.data)
            rospy.loginfo("[TriggerActionNode]: Deleted Trigger: " + msg.data)


if __name__ == "__main__":
    trigger_action_manager_node = TriggerActionNode()

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        rate.sleep()
