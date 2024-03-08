#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from scipy.spatial.transform import Rotation as R

class ModelStatesPublisher:
    def __init__(self):
        rospy.init_node("model_states_publisher")

        self.model_states_pub = rospy.Publisher(
            "/gazebo/model_states", ModelStates, queue_size=1
        )
        # self.fetch_model_state_sub = rospy.Subscriber(
        #     "/vicon/fetch_head", ModelStates, self.fetch_model_state_callback
        # )
        self.objects_model_states_sub = rospy.Subscriber(
            "/vicon/objects", ModelStates, self.objects_model_states_callback
        )

        self.is_start_pose_initialized = False
        self.vicon_to_world_inverse_transform = np.eye(4)
        self.fetch_start_pose = Pose()
        self.model_states = ModelStates()
        self.object_model_states = ModelStates()
        self.fetch_model_state = ModelStates()

    def get_fetch_transform_matrix(self):
        fetch_start_pose = Pose()
        fetch_start_pose.position = Point(535.0299684081835, -54.64279332823344, 0.0)#990.6674009549946)
        fetch_start_pose.orientation = Quaternion(-0.007206383137603431, 0.011539465053337201, 0.03170873773655549, 0.9994045244016496)
        self.fetch_start_pose = fetch_start_pose
        
        transform_matrix = self.get_transform_matrix(self.fetch_start_pose.position, self.fetch_start_pose.orientation)
        self.vicon_to_world_inverse_transform = np.linalg.inv(transform_matrix)
        self.is_start_pose_initialized = True

    def get_transform_matrix(self, position, orientation):
        # Create a 4x4 transformation matrix from position and orientation
        rotation_matrix = np.eye(4)

        # Extract rotation matrix from quaternion
        rotation_matrix[:3, :3] = np.array([
            [1 - 2 * (orientation.y**2 + orientation.z**2), 2 * (orientation.x * orientation.y - orientation.z * orientation.w), 2 * (orientation.x * orientation.z + orientation.y * orientation.w)],
            [2 * (orientation.x * orientation.y + orientation.z * orientation.w), 1 - 2 * (orientation.x**2 + orientation.z**2), 2 * (orientation.y * orientation.z - orientation.x * orientation.w)],
            [2 * (orientation.x * orientation.z - orientation.y * orientation.w), 2 * (orientation.y * orientation.z + orientation.x * orientation.w), 1 - 2 * (orientation.x**2 + orientation.y**2)],
        ])

        translation_matrix = np.eye(4)
        translation_matrix[:3, 3] = [position.x / 1000.0, position.y / 1000.0, position.z / 1000.0]

        transform_matrix = np.dot(rotation_matrix, translation_matrix)

        return transform_matrix
    
    def rotation_matrix_to_quaternion(self, rotation_matrix):
            # Convert rotation matrix to quaternion using scipy
            rotation = R.from_matrix(rotation_matrix)
            quaternion = rotation.as_quat()
            return quaternion

    def vicon_objects_to_world_transform(self):
        if self.object_model_states:
            # Reset the poses list
            model_states_transformed = ModelStates()

            for pose in self.object_model_states.pose:
                # Get the transformation matrix of the object
                object_transform_matrix = self.get_transform_matrix(pose.position, pose.orientation)

                # Transform the object pose to the world coordinate system
                object_to_world_transform = np.dot(self.vicon_to_world_inverse_transform, object_transform_matrix)

                # Extract the position (x, y, z) from the transformed matrix
                transformed_position = object_to_world_transform[:3, 3]

                # Extract the orientation as a rotation matrix from the transformed matrix
                transformed_rotation_matrix = object_to_world_transform[:3, :3]

                # Convert the rotation matrix to quaternion
                transformed_quaternion = self.rotation_matrix_to_quaternion(transformed_rotation_matrix)

                # Create Pose message for the transformed pose
                transformed_pose = Pose()
                transformed_pose.position = Point(*transformed_position)
                transformed_pose.orientation = Quaternion(*transformed_quaternion)

                transformed_no_matrix = Pose()
                transformed_no_matrix.position.x = pose.position.x/1000.0
                transformed_no_matrix.position.y = pose.position.y/1000.0
                transformed_no_matrix.position.z = pose.position.z/1000.0
                transformed_no_matrix.orientation.x = pose.orientation.x
                transformed_no_matrix.orientation.y = pose.orientation.y
                transformed_no_matrix.orientation.z = pose.orientation.z
                transformed_no_matrix.orientation.w = pose.orientation.w
                # Append the transformed pose to self.model_states.pose
                model_states_transformed.pose.append(transformed_no_matrix)
                model_states_transformed.name.append(self.object_model_states.name)

            self.model_states = model_states_transformed
            self.model_states.name = self.object_model_states.name

    def publish(self):
        if self.is_start_pose_initialized:
            self.model_states_pub.publish(self.model_states)

    def fetch_model_state_callback(self, msg):
        self.fetch_model_state = msg
        if not self.is_start_pose_initialized:
            self.get_fetch_transform_matrix()
            self.is_start_pose_initialized = True

    def objects_model_states_callback(self, msg):
        self.object_model_states = msg


if __name__ == "__main__":
    model_states_publisher = ModelStatesPublisher()
    model_states_publisher.get_fetch_transform_matrix()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        model_states_publisher.vicon_objects_to_world_transform()
        model_states_publisher.publish()
        rate.sleep()