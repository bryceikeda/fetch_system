#!/usr/bin/env python3

import rospy
import rospkg
from tf2_ros import TransformListener, Buffer
import tf2_ros
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
import networkx as nx
import matplotlib.pyplot as plt
from scene_graph.srv import QuerySceneGraph, QuerySceneGraphResponse
from shape_msgs.msg import SolidPrimitive
import numpy as np
import transforms3d as tf3d

class SceneGraphNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("scene_graph_node")

        # Initialize empty graph
        self.graph = nx.DiGraph()
        self.node_size = 200
        self.alpha = 0.5
        self.pos_3d = {}
        self.pos_2d = {}
        self.display_in_2d = True

        # Create a service client to call the getSceneObjectsRequest service
        self.get_scene_objects_client = rospy.ServiceProxy(
            "/world_monitor/get_scene_objects", GetPlanningScene
        )

        self.get_scene_objects_client.wait_for_service()

        self.scene_graph_query_service = rospy.Service(
            "/scene_graph/query", QuerySceneGraph, self.handle_scene_graph_query_request
        )

        self.scene_objects = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def handle_scene_graph_query_request(self, req):
        related_nodes = []
        attributes = []

        if req.relationship_type:
            if req.relationship_type == "is_on":
                node_object_is_on = self.get_surface_object_is_on(req.node_name)
                related_nodes = [node_object_is_on]
            elif req.relationship_type == "supports":
                related_nodes = self.get_objects_supported_by_surface(req.node_name)

        if req.attribute_name:
            if req.attribute_name == "subframe_names":
                attributes = self.graph.nodes[req.node_name]["subframe_names"]
            else:
                attributes = self.graph.nodes[req.node_name][req.attribute_name]
                if not isinstance(attributes, list):
                    attributes = [attributes]

        res = QuerySceneGraphResponse()
        res.related_nodes = related_nodes
        res.attributes = attributes  # Fixed the typo in 'attributes'
        return res

    def initialize_scene_graph(self):
        while self.scene_objects == None:
            self.get_scene_objects()
            rospy.sleep(5)

        for collision_object in self.scene_objects.world.collision_objects:
            if collision_object.type.key == "surface":
                dimensions = collision_object.primitives[0].dimensions
                subframe_names = [name for name in collision_object.subframe_names]
                attributes = {
                    "type": "surface",
                    "subframe_names": subframe_names,
                }
                # Use the first pose in 'primitive_poses' as the position
                position = (
                    collision_object.pose.position.x
                    + collision_object.primitive_poses[0].position.x,
                    collision_object.pose.position.y
                    + collision_object.primitive_poses[0].position.y,
                    collision_object.pose.position.z
                    + collision_object.primitive_poses[0].position.z,
                )
                orientation = (
                    collision_object.pose.orientation.x,
                    collision_object.pose.orientation.y,
                    collision_object.pose.orientation.z,
                    collision_object.pose.orientation.w
                )
                if collision_object.primitives[0].type == 1:
                    shape = "box"
                else:
                    shape = "cylinder"
            else:
                if len(collision_object.primitives) == 0:
                    dimensions = (0.18, 0.0)
                    shape = "cylinder"
                else: 
                    dimensions = collision_object.primitives[0].dimensions
                    if collision_object.primitives[0].type == 1:
                        shape = "box"
                    else:
                        shape = "cylinder"
                attributes = {"type": "object"}
                # Use the first pose in 'primitive_poses' as the position
                position = (
                    collision_object.pose.position.x,
                    collision_object.pose.position.y,
                    collision_object.pose.position.z,
                )
                orientation = (
                    collision_object.pose.orientation.x,
                    collision_object.pose.orientation.y,
                    collision_object.pose.orientation.z,
                    collision_object.pose.orientation.w,
                )

            attributes.update(
                {"position": position, "orientation": orientation, "dimensions": dimensions, "shape": shape}
            )

            # Add the object node to the graph
            self.add_object_node(collision_object.id, attributes=attributes)

    def get_scene_objects(self):
        try:
            response = self.get_scene_objects_client.call(GetPlanningSceneRequest())
            self.scene_objects = response.scene
        except rospy.ServiceException as e:
            rospy.sleep(5)

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
        return (
            (pos1[0] - pos2[0]) ** 2
            + (pos1[1] - pos2[1]) ** 2
            + (pos1[2] - pos2[2]) ** 2
        ) ** 0.5

    def get_surface_object_is_on(self, object_node):
        for surface_node, attributes in self.graph.nodes(data=True):
            if attributes.get("type") == "surface":
                supported_nodes = self.get_objects_supported_by_surface(
                    surface_node
                )
                for supported_node in supported_nodes:
                    if supported_node == object_node:
                        return surface_node
        return None

    def get_objects_supported_by_surface(self, surface_node):
        supported_nodes = []

        # Iterate through all edges connected to the support surface node
        for _, other_node, edge_data in self.graph.out_edges(
            surface_node, data=True
        ):
            # Check if the edge represents a "supports" relationship
            if edge_data.get("label") == "supports":
                supported_nodes.append(other_node)
        return supported_nodes

    def calculate_supports_relationship(self):
        # Iterate through each node in the graph
        for node, attributes in self.graph.nodes(data=True):
            # Check if the node represents a support surface (e.g., a table)
            if attributes.get("type") == "surface":
                surface_node = node
                # Iterate through all nodes in the graph
                for other_node, other_attributes in self.graph.nodes(data=True):
                    # Check if the other node is an object and not the same as the support surface
                    if (
                        other_attributes.get("type", "") == "object"
                        and other_node != surface_node
                    ):
                        # Check if the object is on the support surface
                        if self.is_object_on_surface(
                            surface_node, other_node
                        ):
                            # Add an edge representing the "supports" relationship with 'label'
                            self.graph.add_edge(
                                surface_node, other_node, label="supports"
                            )
                        elif self.graph.has_edge(surface_node, other_node):
                            self.graph.remove_edge(surface_node, other_node)
                            print(
                                f"Removed edge between {surface_node} and {other_node}"
                            )

    def get_inverse_transform_matrix(self, position, orientation):
        inverse_translation_matrix = np.array([
            [1, 0, 0, -position[0]],
            [0, 1, 0, -position[1]],
            [0, 0, 1, -position[2]],
            [0, 0, 0, 1]
        ])

        rotation_matrix = tf3d.quaternions.quat2mat((orientation[3], orientation[0], orientation[1], orientation[2]))
        expand_rotation_matrix = np.eye(4)
        expand_rotation_matrix[:3, :3] = rotation_matrix
        inverse_transform_matrix = np.dot(np.linalg.inv(expand_rotation_matrix), inverse_translation_matrix)
        return inverse_transform_matrix

    def transform_object(self, object_position, parent_position, parent_orientation):
        # Get transformation matrices for the parent and child
        parent_inverse_transform_matrix = self.get_inverse_transform_matrix(parent_position, parent_orientation)

        object_point = np.array([*object_position, 1])
        # Combine transformations: child relative to parent
        object_transformed_position = np.dot(parent_inverse_transform_matrix, object_point)

        return object_transformed_position[:3]
    
    def is_object_on_surface(self, surface_node, object_node):
        # Get positions and dimensions of the object and support surface nodes
        object_position = self.graph.nodes[object_node]["position"]
        object_shape = self.graph.nodes[object_node]["shape"]
        object_dimensions = self.graph.nodes[object_node]["dimensions"]

        surface_position = self.graph.nodes[surface_node]["position"]
        surface_orientation = self.graph.nodes[surface_node]["orientation"]
        surface_dimensions = self.graph.nodes[surface_node][
            "dimensions"
        ]

        object_position = self.transform_object(object_position, surface_position, surface_orientation)
    
        if object_shape == "box":
            # Check if the object is above the support surface but below a little above the middle of the object
            is_above = (
                0
                < object_position[2] # move object and table to 0
                < surface_dimensions[2] / 2
                + object_dimensions[SolidPrimitive.BOX_Z] / 2
                + 0.05
            )
        elif object_shape == "cylinder":
            is_above = (
                0
                < object_position[2]
                < surface_dimensions[2] / 2
                + object_dimensions[SolidPrimitive.CYLINDER_HEIGHT] / 2
                + 0.05
            )

        is_within_x = (
            -surface_dimensions[0] / 2
            < object_position[0]
            < surface_dimensions[0] / 2
        )
        is_within_y = (
            -surface_dimensions[1] / 2
            < object_position[1]
            < surface_dimensions[1] / 2
        )

        return is_above and is_within_x and is_within_y

    def draw(self):
        self.calculate_positions()

        fig, ax = plt.subplots()

        # Draw edges
        nx.draw_networkx_edges(
            self.graph, self.pos_2d, ax=ax, alpha=self.alpha, arrowsize=20
        )

        # Draw nodes with transparency and different colors
        nx.draw_networkx_nodes(
            self.graph, self.pos_2d, ax=ax, alpha=self.alpha, node_color="lightgreen"
        )

        # Draw node labels
        nx.draw_networkx_labels(self.graph, self.pos_2d, ax=ax, font_weight="bold")

        # Draw edge labels
        edge_labels = nx.get_edge_attributes(self.graph, "label")
        nx.draw_networkx_edge_labels(
            self.graph, self.pos_2d, edge_labels=edge_labels, ax=ax
        )

        plt.show()

if __name__ == "__main__":
    scene_graph = SceneGraphNode()
    rate = rospy.Rate(1)
    rospy.sleep(3)
    # Add objects to the scene graph with positions
    scene_graph.initialize_scene_graph()
    
    while not rospy.is_shutdown():
        scene_graph.calculate_supports_relationship()
        scene_graph.draw()
        rate.sleep()
