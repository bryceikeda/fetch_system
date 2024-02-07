#!/usr/bin/env python3

import yaml
import rospy
import rospkg
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion, Twist

class ModelStatesPublisher:
    def __init__(self):
        rospy.init_node('model_states_publisher')

        self.model_states = ModelStates()
        self.model_states_pub = rospy.Publisher('/gazebo/model_states', ModelStates, queue_size=1)

    def model_states_publisher(self):
        self.model_states_pub.publish(self.model_states)

    def read_yaml_file(self, file_path):
        try:
            # Open the YAML file for reading
            with open(file_path, 'r') as file:
                # Load the YAML data
                yaml_data = yaml.safe_load(file)

                # Iterate through each object in the YAML data
                for item in yaml_data:
                    # Extract relevant attributes
                    name = item['name']
                    position = item['position']

                    pose = Pose()
                    pose.position.x = float(position[0])
                    pose.position.y = float(position[1])
                    pose.position.z = float(position[2])
                    pose.orientation.x = float(position[3])
                    pose.orientation.y = float(position[4])
                    pose.orientation.z = float(position[5])
                    pose.orientation.w = float(position[6])

                    self.model_states.name.append(name)
                    self.model_states.pose.append(pose)
                    self.model_states.twist.append(Twist())

        except Exception as e:
            print(f"Error reading YAML file: {e}")
            return None

if __name__ == '__main__':
    msp = ModelStatesPublisher()

    model_states_file_path = rospkg.RosPack().get_path('simulator') + "/config/model_states.yaml"
    msp.read_yaml_file(model_states_file_path)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msp.model_states_publisher()
        rate.sleep()
