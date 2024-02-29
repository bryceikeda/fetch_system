#!/usr/bin/env python3

import rospy
import rospkg
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion, Twist
from tf2_ros import TransformListener; 
from moveit_msgs.srv import GetPlanningScene;
from moveit_msgs.msg import PlanningSceneWorld, CollisionObject
import networkx as nx
import matplotlib.pyplot as plt
import random

class SceneGraphNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('scene_graph_node')

        # Initialize empty graph
        self.graph = nx.Graph()
        self.pos = None
        self.scene_objects = PlanningSceneWorld()

        # Create a service client to call the getSceneObjectsRequest service
        #rospy.wait_for_service('/world_monitor/get_scene_objects')
        self.get_scene_objects_client = rospy.ServiceProxy('/world_monitor/get_scene_objects', GetPlanningScene)

    def get_scene_objects(self):
        try:
            self.get_scene_objects_client(self.scene_objects)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def add_node(self, node, attribute, value):
        self.graph.add_node(node, attribute=value)

    def add_edge(self, node1, node2, label=None):
        self.graph.add_edge(node1, node2, label=label)

    def calculate_positions(self):
        self.pos = nx.spring_layout(self.graph)
        return self.pos

if __name__ == '__main__':
    scene_graph_node = SceneGraphNode()
    
    #rospy.sleep(5)

    rate = rospy.Rate(10) 

    # Add nodes and edges
    scene_graph_node.add_node("Node 1", 'IsManipulable', True)
    scene_graph_node.add_node("Node 2")
    scene_graph_node.add_edge("Node 1", "Node 2", "Is a")

    i = 0
    while not rospy.is_shutdown():

        if i < 10: 
            new_node = f"Node {3 + _}"
            scene_graph_node.add_node(new_node)
            random_node = random.choice(list(scene_graph_node.graph.nodes))
            scene_graph_node.add_edge(new_node, random_node, f"Edge {_}")
            pos = scene_graph_node.update_visualization()
            # Plot outside the class
            scene_graph_node.draw_graph()
            nx.draw(scene_graph_node.graph, pos=pos, with_labels=True)
            edge_labels = nx.get_edge_attributes(scene_graph_node.graph, 'label')
            nx.draw_networkx_edge_labels(scene_graph_node.graph, pos, edge_labels=edge_labels)
            plt.pause(0.1)

        plt.show()
        i += 1    
        rate.sleep()





