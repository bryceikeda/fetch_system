#!/usr/bin/env python3

import socket
import rospy
import json
import numpy as np
from gazebo_msgs.msg import ModelState, ModelStates

## Global variables VICON
port_VICON = 12345 # define port number to connect to socket
IP_VICON = '152.23.114.34' # IP address of the computer running VICON tracker
iter_VICON = 10
packet_size = 4096
fetch_head = "Fetch head"

# Get object locations and orientations from VICON averaged over iter_VICON
def getVICON(iter_VICON):
    object_locations = {}
    object_orientations = {}
    object_names = None

    for _ in range(iter_VICON):
        # create socket
        s = socket.socket()

        try:
            # connect to VICON server
            s.connect((IP_VICON, port_VICON))

            # receive data from VICON
            packet = s.recv(packet_size).decode("utf-8")
            # print("VICON received: ", packet, "\n\n")
        except Exception as e:
            print("VICON error: ")
            raise e

        # close connection to VICON
        s.close()

        packet = json.loads(packet)
        object_names = packet.keys()
        for obj_name in object_names:
            obj_inst = packet[obj_name][obj_name]
            # assert obj_inst["global_tx"][1]
            # assert obj_inst["global_rot"][1]

            if obj_name in object_locations:
                object_locations[obj_name] += np.array(obj_inst["global_tx"][0])
                object_orientations[obj_name] += np.array(obj_inst["global_rot"][0])
            else:
                object_locations[obj_name] = np.array(obj_inst["global_tx"][0])
                object_orientations[obj_name] = np.array(obj_inst["global_rot"][0])

    for obj_name in object_names:
        # print(obj_name)
        # print("Locations")
        # print(object_locations[obj_name])
        # print("orientations")
        # print(object_orientations[obj_name])
        object_locations[obj_name] = object_locations[obj_name] / iter_VICON
        object_orientations[obj_name] = object_orientations[obj_name] / iter_VICON



    fetch_msg = ModelState()
    fetch_msg.model_name = fetch_head

    fetch_msg.pose.position.x = object_locations[fetch_head][0]
    fetch_msg.pose.position.y = object_locations[fetch_head][1]
    fetch_msg.pose.position.z = object_locations[fetch_head][2]

    fetch_msg.pose.orientation.x = object_orientations[fetch_head][0]
    fetch_msg.pose.orientation.y = object_orientations[fetch_head][1]
    fetch_msg.pose.orientation.z = object_orientations[fetch_head][2]
    fetch_msg.pose.orientation.w = object_orientations[fetch_head][3]

    objects_msg = ModelStates()
    for obj_name in object_names:
        if obj_name == fetch_head:
            continue 
        msg = ModelState()
        msg.model_name = obj_name

        msg.pose.position.x = object_locations[obj_name][0]
        msg.pose.position.y = object_locations[obj_name][1]
        msg.pose.position.z = object_locations[obj_name][2]

        msg.pose.orientation.x = object_orientations[obj_name][0]
        msg.pose.orientation.y = object_orientations[obj_name][1]
        msg.pose.orientation.z = object_orientations[obj_name][2]
        msg.pose.orientation.w = object_orientations[obj_name][3]

        objects_msg.name.append(msg.model_name)
        objects_msg.pose.append(msg.pose)

    return fetch_msg, objects_msg

if __name__ == '__main__':
    # init ROS
    rospy.init_node('VICON_to_ROS', anonymous=True)
    fetch_pub = rospy.Publisher('vicon/fetch_head', ModelState, queue_size=1)
    objects_pub = rospy.Publisher('vicon/objects', ModelStates, queue_size=1)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # get object locations and orientations from VICON averaged over iter_VICON
        fetch_msg, objects_msg = getVICON(iter_VICON)

        fetch_pub.publish(fetch_msg)
        objects_pub.publish(objects_msg)
        
        # print(fetch_msg)
        # print("-----------------------------------------------------")
        # print(objects_msg)
        # print()
        rate.sleep()
