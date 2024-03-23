#!/usr/bin/env python3

import socket
import json
import numpy as np
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import PoseStamped, Pose
import math
import time
from vicon_receiver.transform_helpers import transform_object_to_ref_frame
from nav_msgs.msg import Odometry

class ViconDataReceiver:
    def __init__(self, ip, port, iter_vicon, packet_size, object_reference_frame_name):
        self.ip = ip
        self.port = port
        self.iter_vicon = iter_vicon
        self.packet_size = packet_size
        self.object_reference_frame_name = object_reference_frame_name

    def connect_to_vicon_server(self):
        s = socket.socket()
        try:
            s.connect((self.ip, self.port))
            packet = s.recv(self.packet_size).decode("utf-8")
            s.close()
            return packet
        except Exception as e:
            print("VICON error: ")
            raise e

    def parse_vicon_data(self, packet):
        vicon_data = json.loads(packet)
        object_locations = {}
        object_orientations = {}
        object_names = list(vicon_data.keys())
        for obj_name in object_names:
            obj_inst = vicon_data[obj_name][obj_name]
            object_locations[obj_name] = np.array(obj_inst["global_tx"][0])
            object_orientations[obj_name] = np.array(obj_inst["global_rot"][0])
        return object_names, object_locations, object_orientations

    def average_data(self, data_dict):
        for obj_name, data in data_dict.items():
            data_dict[obj_name] = np.mean(data, axis=0)
        return data_dict

    def normalize_quaternion(self, quaternion):
        x, y, z, w = quaternion
        magnitude = math.sqrt(x ** 2 + y ** 2 + z ** 2 + w ** 2)
        return x / magnitude, y / magnitude, z / magnitude, w / magnitude

    def get_data(self):
        object_names = None
        object_locations = {}
        object_orientations = {}
        object_model_states = ModelStates()
        object_reference_frame_pose = Pose()
        odometry = Odometry()

        for _ in range(self.iter_vicon):
            packet = self.connect_to_vicon_server()
            names, locations, orientations = self.parse_vicon_data(packet)
            if object_names is None:
                object_names = names
            for obj_name in object_names:
                object_locations.setdefault(obj_name, []).append(locations[obj_name])
                object_orientations.setdefault(obj_name, []).append(orientations[obj_name])

        object_locations = self.average_data(object_locations)
        object_orientations = self.average_data(object_orientations)

        ref_frame_loc = object_locations[self.object_reference_frame_name]
        ref_frame_orient = object_orientations[self.object_reference_frame_name]

        object_reference_frame_pose.position.x = ref_frame_loc[0] / 1000.0 
        object_reference_frame_pose.position.y = ref_frame_loc[1] / 1000.0 
        object_reference_frame_pose.position.z = 0.0

        norm_quat = self.normalize_quaternion(ref_frame_orient)
        object_reference_frame_pose.orientation.x = norm_quat[0]
        object_reference_frame_pose.orientation.y = norm_quat[1]
        object_reference_frame_pose.orientation.z = norm_quat[2]
        object_reference_frame_pose.orientation.w = norm_quat[3]

        for obj_name in object_names:
            if obj_name == self.object_reference_frame_name:
                odometry.header.frame_id = obj_name
                odometry.pose = object_reference_frame_pose

            pose = Pose()
            pose.position.x = object_locations[obj_name][0] / 1000.0 
            pose.position.y = object_locations[obj_name][1] / 1000.0 
            pose.position.z = object_locations[obj_name][2] / 1000.0 
            pose.orientation.x = object_orientations[obj_name][0]
            pose.orientation.y = object_orientations[obj_name][1]
            pose.orientation.z = object_orientations[obj_name][2]
            pose.orientation.w = object_orientations[obj_name][3]

            transformed_pose = transform_object_to_ref_frame(pose, object_reference_frame_pose)
            object_model_states.name.append(obj_name)
            object_model_states.pose.append(transformed_pose)
        return odometry, object_model_states
    
if __name__ == '__main__':
    vicon_receiver = ViconDataReceiver(ip='192.168.1.12', port=12345, iter_vicon=10, packet_size=4096, object_reference_frame_name="base_link")
    while True:
        object_model_states = vicon_receiver.get_data()
        print(object_model_states)
        print("\n")
        time.sleep(1)