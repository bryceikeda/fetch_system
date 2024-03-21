#!/usr/bin/env python3

import rospy
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
import networkx as nx
import matplotlib.pyplot as plt
from scene_graph.srv import QuerySceneGraph, QuerySceneGraphResponse
from scene_graph.msg import UnityRelationships
from std_msgs.msg import String
from scene_graph.scene_graph import SceneGraph


class SceneGraphNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("scene_graph_node")
        
        # Initialize the scene graph
        self.scene_graph = SceneGraph()

        # Initialize empty graph attributes
        self.node_size = 200
        self.alpha = 0.5
        self.display_in_2d = True
        self.pos_3d = {}
        self.pos_2d = {}
        self.scene_objects = None
        self.unity_scene_relationships = UnityRelationships()
        self.previous_node_count = 0
        
        # Subscribers
        self.unity_relationships_subscriber = rospy.Subscriber(
            "/unity/scene_relationships",
            UnityRelationships,
            self.handle_scene_relationships
        )
        self.delete_trigger_subscriber = rospy.Subscriber(
            "/unity/delete_trigger_node",
            String,
            self.handle_delete_trigger
        )

        # Service clients
        self.get_scene_objects_client = rospy.ServiceProxy(
            "/world_monitor/get_scene_objects",
            GetPlanningScene
        )
        self.get_scene_objects_client.wait_for_service()

        # Scene graph query service
        self.scene_graph_query_service = rospy.Service(
            "/scene_graph/query",
            QuerySceneGraph,
            self.handle_scene_graph_query_request
        )

        # Timer for graph updates
        self.update_timer = rospy.Timer(
            rospy.Duration(0.5),
            self.update_graph_callback
        )

        # Plot attributes
        self.fig, self.ax = plt.subplots()

    def handle_scene_relationships(self, msg):
        self.unity_scene_relationships = msg

    def update_graph_callback(self, event):
        self.scene_graph.clear_graph_edges()
        self.scene_graph.calculate_supports_relationship()
        self.scene_graph.add_unity_scene_relationships(self.unity_scene_relationships)
        self.draw()

    def handle_delete_trigger(self, msg):
        self.scene_graph.delete_trigger(msg.data)

    def handle_scene_graph_query_request(self, req):
        res = QuerySceneGraphResponse()
        res.related_nodes, res.attributes = self.scene_graph.query_scene_graph(
            req.relationship_type, req.attribute_name, req.node_name
        )
        return res

    def initialize_scene_graph(self):
        while self.scene_objects == None:
            self.get_scene_objects()
            rospy.sleep(3)
        
        self.scene_graph.add_node("user")

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
                    collision_object.pose.orientation.w,
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
                {
                    "position": position,
                    "orientation": orientation,
                    "dimensions": dimensions,
                    "shape": shape,
                }
            )

            # Add the object node to the graph
            self.scene_graph.add_node(collision_object.id, attributes=attributes)

    def extract_collision_object_attributes(self, collision_object):
        attributes = {}
        attributes["position"] = (
            collision_object.pose.position.x,
            collision_object.pose.position.y,
            collision_object.pose.position.z,
        )
        attributes["orientation"] = (
            collision_object.pose.orientation.x,
            collision_object.pose.orientation.y,
            collision_object.pose.orientation.z,
            collision_object.pose.orientation.w,
        )
        if collision_object.type.key == "surface":
            dimensions = collision_object.primitives[0].dimensions
            attributes["subframe_names"] = [
                name for name in collision_object.subframe_names
            ]
            if collision_object.primitives[0].type == 1:
                attributes["shape"] = "box"
            else:
                attributes["shape"] = "cylinder"
        else:
            if len(collision_object.primitives) == 0:
                dimensions = (0.18, 0.0)
                attributes["shape"] = "cylinder"
            else:
                dimensions = collision_object.primitives[0].dimensions
                if collision_object.primitives[0].type == 1:
                    attributes["shape"] = "box"
                else:
                    attributes["shape"] = "cylinder"
        attributes["dimensions"] = dimensions
        attributes["type"] = (
            "surface" if collision_object.type.key == "surface" else "object"
        )
        return attributes

    def get_scene_objects(self):
        try:
            response = self.get_scene_objects_client.call(GetPlanningSceneRequest())
            self.scene_objects = response.scene
        except rospy.ServiceException as e:
            rospy.sleep(5)

    def calculate_positions(self):
        self.pos_3d = nx.fruchterman_reingold_layout(self.scene_graph.graph, dim=3)
        if self.display_in_2d:
            # Extract 2D coordinates
            self.pos_2d = {node: (x, y) for node, (x, y, z) in self.pos_3d.items()}

    def draw(self):
        node_count = self.scene_graph.graph.number_of_nodes()
        if node_count != self.previous_node_count:
            self.previous_node_count = node_count
            self.calculate_positions()

        self.ax.clear()
        # Draw edges
        nx.draw_networkx_edges(
            self.scene_graph.graph,
            self.pos_2d,
            ax=self.ax,
            alpha=self.alpha,
            arrowsize=20,
        )

        # Draw nodes with transparency and different colors
        nx.draw_networkx_nodes(
            self.scene_graph.graph,
            self.pos_2d,
            ax=self.ax,
            alpha=self.alpha,
            node_color="lightgreen",
        )

        # Draw node labels
        nx.draw_networkx_labels(
            self.scene_graph.graph, self.pos_2d, ax=self.ax, font_weight="bold"
        )

        # Draw edge labels
        edge_labels = nx.get_edge_attributes(self.scene_graph.graph, "label")
        nx.draw_networkx_edge_labels(
            self.scene_graph.graph, self.pos_2d, edge_labels=edge_labels, ax=self.ax
        )
        plt.draw()


if __name__ == "__main__":
    scene_graph = SceneGraphNode()
    rate = rospy.Rate(40)
    rospy.sleep(3)
    # Add objects to the scene graph with positions
    scene_graph.initialize_scene_graph()
    scene_graph.draw()
    plt.show(block=True)
