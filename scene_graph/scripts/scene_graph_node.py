#!/usr/bin/env python3

import rospy
import rospkg
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion, Twist
from tf2_ros import TransformListener, Buffer
import tf2_ros
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from moveit_msgs.msg import PlanningSceneWorld
import networkx as nx
import matplotlib.pyplot as plt
import itertools
from scene_graph.srv import QuerySceneGraph, QuerySceneGraphResponse

class SceneGraphNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('scene_graph_node')

        # Initialize empty graph
        self.graph = nx.DiGraph()
        self.node_size = 200
        self.alpha = .5
        self.pos_3d = {}
        self.pos_2d = {}
        self.display_in_2d = True
        
        # Create a service client to call the getSceneObjectsRequest service
        self.get_scene_objects_client = rospy.ServiceProxy('/world_monitor/get_scene_objects', GetPlanningScene)
        self.scene_graph_query_service = rospy.Service('/scene_graph/query', QuerySceneGraph, self.handle_scene_graph_query_request)

        self.scene_objects = PlanningSceneWorld()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def handle_scene_graph_query_request(self, req):
        related_nodes = []
        attributes = None

        if req.relationship_type != None: 
            if req.relationship_type == "is_on":
                node_object_is_on = self.get_surface_object_is_on(req.node_name)
                related_nodes = [node_object_is_on]
            elif req.relationship_type == "supports":
                related_nodes = self.get_objects_supported_by_surface(req.node_name)

        if req.attribute_name != None:
            if req.attribute_name != "subframe_names":
                attributes = self.graph.nodes[req.node_name][req.attribute_name]
                if isinstance(attributes, list):
                    attributes = attributes
                else:
                    attributes = [attributes]

            if req.attribute_name == "subframe_names":
                attributes = self.graph.nodes[req.node_name]["subframe_names"]

        res = QuerySceneGraphResponse()
        res.related_nodes = related_nodes
        res.attributes = attributes  # Fixed the typo in 'attributes'
        return res

    def initialize_scene_graph(self):
        self.get_scene_objects()

        for collision_object in self.scene_objects.world.collision_objects:
            if collision_object.type.key == "mesh":
                dimensions = (0.06, 0.06, 0.06)
                attributes = {"type": "object"}
                # Use the first pose in 'primitive_poses' as the position
                position = (collision_object.pose.position.x,
                            collision_object.pose.position.y,
                            collision_object.pose.position.z)
                # Add the object node to the graph
            elif collision_object.type.key == "static":
                dimensions = collision_object.primitives[0].dimensions
                subframe_names = [name for name in collision_object.subframe_names]
                attributes = {"type": "support_surface", "subframe_names": subframe_names}
                # Use the first pose in 'primitive_poses' as the position
                position = (collision_object.pose.position.x + collision_object.primitive_poses[0].position.x,
                            collision_object.pose.position.y + collision_object.primitive_poses[0].position.y,
                            collision_object.pose.position.z + collision_object.primitive_poses[0].position.z)
            else:
                dimensions = collision_object.primitives[0].dimensions
                attributes = {"type": "object"}
                # Use the first pose in 'primitive_poses' as the position
                position = (collision_object.pose.position.x,
                            collision_object.pose.position.y,
                            collision_object.pose.position.z)

            if collision_object.primitives[0].type == 1:
                shape = "box"
            else:
                shape = "cylinder"

            attributes.update({"position": position, "dimensions": dimensions, "shape": shape})

            # Add the object node to the graph
            self.add_object_node(collision_object.id, attributes=attributes)

    def get_scene_objects(self):
        try:
            response = self.get_scene_objects_client.call(GetPlanningSceneRequest())
            self.scene_objects = response.scene
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def add_object_node(self, name, attributes=None):
            if attributes is None:
                attributes = {}

            self.graph.add_node(name, **attributes)

    def calculate_positions(self):
        self.pos_3d = nx.fruchterman_reingold_layout(self.graph, dim=3)
        if self.display_in_2d:
            # Extract 2D coordinates
            self.pos_2d = {node: (x, y) for node, (x, y, z) in self.pos_3d.items()}

    def calculate_distance(self, pos1, pos2):
        return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2 + (pos1[2] - pos2[2])**2)**0.5
    
    def calculate_supports_relationship(self):
        # Iterate through each node in the graph
        for node, attributes in self.graph.nodes(data=True):
            # Check if the node represents a support surface (e.g., a table)
            if attributes.get("type") == "support_surface":
                support_surface_node = node
                # Iterate through all nodes in the graph
                for other_node, other_attributes in self.graph.nodes(data=True):
                    # Check if the other node is an object and not the same as the support surface
                    if other_attributes.get("type", "") == "object" and other_node != support_surface_node:
                        # Check if the object is on the support surface
                        if self.is_object_on_support_surface(support_surface_node, other_node):
                            # Add an edge representing the "supports" relationship with 'label'
                            self.graph.add_edge(support_surface_node, other_node, label="supports")
                        elif self.graph.has_edge(support_surface_node, other_node):
                            self.graph.remove_edge(support_surface_node, other_node)
                            print(f"Removed edge between {support_surface_node} and {other_node}")

    def get_surface_object_is_on(self, object_node):
        for support_surface_node, attributes in self.graph.nodes(data=True):
            if attributes.get("type") == "support_surface":
                supported_nodes = self.get_objects_supported_by_surface(support_surface_node)
                for supported_node in supported_nodes:
                    if supported_node == object_node:
                        return support_surface_node
        return None

    def get_objects_supported_by_surface(self, support_surface_node):
        supported_nodes = []

        # Iterate through all edges connected to the support surface node
        for _, other_node, edge_data in self.graph.out_edges(support_surface_node, data=True):
            # Check if the edge represents a "supports" relationship
            if edge_data.get("label") == "supports":
                supported_nodes.append(other_node)
        return supported_nodes

    def is_object_on_support_surface(self, support_surface_node, object_node):
        # Get positions and dimensions of the object and support surface nodes
        object_position = self.graph.nodes[object_node]["position"]
        object_shape = self.graph.nodes[object_node]["shape"]

        support_surface_position = self.graph.nodes[support_surface_node]["position"]
        support_surface_dimensions = self.graph.nodes[support_surface_node]["dimensions"]
        if object_shape == "box":
            # Check if the object is above the support surface
            is_above = object_position[2] < support_surface_position[2] + support_surface_dimensions[2] / 2 + .05
        elif object_shape == "cylinder":
            is_above = object_position[2] < support_surface_position[2] + support_surface_dimensions[0] / 2 + .05

        is_within_x = support_surface_position[0] - support_surface_dimensions[0] / 2 < object_position[0] < support_surface_position[0] + support_surface_dimensions[0] / 2
        is_within_y = support_surface_position[1] - support_surface_dimensions[1] / 2 < object_position[1] < support_surface_position[1] + support_surface_dimensions[1] / 2

        return is_above and is_within_x and is_within_y
    
    def draw(self):
        self.calculate_positions()

        fig, ax = plt.subplots()

        # Draw edges
        nx.draw_networkx_edges(self.graph, self.pos_2d,  ax=ax, alpha=self.alpha, arrowsize=20)

        # Draw nodes with transparency and different colors
        nx.draw_networkx_nodes(self.graph, self.pos_2d, ax=ax, alpha=self.alpha, node_color='lightgreen')

        # Draw node labels
        nx.draw_networkx_labels(self.graph, self.pos_2d, ax=ax, font_weight='bold')
       
        # Draw edge labels
        edge_labels = nx.get_edge_attributes(self.graph, 'label')
        nx.draw_networkx_edge_labels(self.graph, self.pos_2d, edge_labels=edge_labels, ax=ax)

        plt.show()

if __name__ == '__main__':
    scene_graph = SceneGraphNode()
    
    rate = rospy.Rate(10) 

    # Add objects to the scene graph with positions
    scene_graph.initialize_scene_graph()
    scene_graph.calculate_supports_relationship()
    while not rospy.is_shutdown():
        scene_graph.draw()
        rate.sleep()
 