#!/usr/bin/env python3

import networkx as nx
from shape_msgs.msg import SolidPrimitive
import numpy as np
import transforms3d as tf3d


class SceneGraph:
    def __init__(self):
        # Initialize empty graph
        self.graph = nx.DiGraph()

    def clear_graph_edges(self):
        edges_to_remove = list(self.graph.edges())
        self.graph.remove_edges_from(edges_to_remove)

    def add_unity_scene_relationships(self, unity_scene_relationships):
        for relationship in unity_scene_relationships.relationships:
            if not self.graph.has_node(relationship.zone):
                self.add_node(relationship.zone)
            for node in relationship.node_objects:
                self.graph.add_edge(
                    relationship.zone, node, label=relationship.relationship
                )

    def query_scene_graph(self, relationship_type, attribute_name, node_name):
        related_nodes = []
        attributes = []

        if relationship_type:
            if relationship_type == "is_on":
                node_object_is_on = self.get_surface_object_is_on(node_name)
                related_nodes = [node_object_is_on]
            else:
                related_nodes = self.get_objects_with_relationship(relationship_type)

        if attribute_name:
            attributes = self.graph.nodes[node_name][attribute_name]
            if not isinstance(attributes, list):
                attributes = [attributes]

        return [related_nodes, attributes]

    def delete_trigger(self, node_name):
        if self.graph.has_node(node_name):
            self.graph.remove_node(node_name)

    def add_node(self, name, attributes=None):
        if attributes is None:
            attributes = {}
        self.graph.add_node(name, **attributes)

    def calculate_distance(self, pos1, pos2):
        return (
            (pos1[0] - pos2[0]) ** 2
            + (pos1[1] - pos2[1]) ** 2
            + (pos1[2] - pos2[2]) ** 2
        ) ** 0.5

    def get_surface_object_is_on(self, object_node):
        for surface_node in self.graph.nodes():
            attributes = self.graph.nodes[surface_node]
            if attributes.get("type") == "surface":
                supported_nodes = self.get_objects_with_relationship(
                    surface_node, "supports"
                )
                if object_node in supported_nodes:
                    return surface_node
        return None

    def get_objects_with_relationship(self, node_name, relationship):
        return [
            related_node
            for _, related_node, edge_data in self.graph.out_edges(node_name, data=True)
            if edge_data.get("label") == relationship
        ]

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
                        if self.is_object_on_surface(surface_node, other_node):
                            # Add an edge representing the "supports" relationship with 'label'
                            self.graph.add_edge(
                                surface_node, other_node, label="supports"
                            )

    def get_inverse_transform_matrix(self, position, orientation):
        inverse_translation_matrix = np.array(
            [
                [1, 0, 0, -position[0]],
                [0, 1, 0, -position[1]],
                [0, 0, 1, -position[2]],
                [0, 0, 0, 1],
            ]
        )

        rotation_matrix = tf3d.quaternions.quat2mat(
            (orientation[3], orientation[0], orientation[1], orientation[2])
        )
        expand_rotation_matrix = np.eye(4)
        expand_rotation_matrix[:3, :3] = rotation_matrix
        inverse_transform_matrix = np.dot(
            np.linalg.inv(expand_rotation_matrix), inverse_translation_matrix
        )
        return inverse_transform_matrix

    def transform_object(self, object_position, parent_position, parent_orientation):
        # Get transformation matrices for the parent and child
        parent_inverse_transform_matrix = self.get_inverse_transform_matrix(
            parent_position, parent_orientation
        )

        object_point = np.array([*object_position, 1])
        # Combine transformations: child relative to parent
        object_transformed_position = np.dot(
            parent_inverse_transform_matrix, object_point
        )

        return object_transformed_position[:3]

    def is_object_on_surface(self, surface_node, object_node):
        # Get positions and dimensions of the object and support surface nodes
        object_position = self.graph.nodes[object_node]["position"]
        object_shape = self.graph.nodes[object_node]["shape"]
        object_dimensions = self.graph.nodes[object_node]["dimensions"]

        surface_position = self.graph.nodes[surface_node]["position"]
        surface_orientation = self.graph.nodes[surface_node]["orientation"]
        surface_dimensions = self.graph.nodes[surface_node]["dimensions"]

        object_position = self.transform_object(
            object_position, surface_position, surface_orientation
        )

        if object_shape == "box":
            # Check if the object is above the support surface but below a little above the middle of the object
            is_above = (
                0
                < object_position[2]  # move object and table to 0
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
            -surface_dimensions[0] / 2 < object_position[0] < surface_dimensions[0] / 2
        )
        is_within_y = (
            -surface_dimensions[1] / 2 < object_position[1] < surface_dimensions[1] / 2
        )

        return is_above and is_within_x and is_within_y
