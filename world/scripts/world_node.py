#!/usr/bin/env python3

import yaml
import rospy
import rospkg
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, CollisionObject, ObjectColor
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Trigger, TriggerResponse
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from typing import Dict

class CollisionObjectBuilder:
    def __init__(self, name, reference_frame, shape, dimensions, offset):
        self.co = CollisionObject()
        self.co.header.frame_id = reference_frame
        self.co.id = name

        solid_primitive = SolidPrimitive()
        solid_primitive.type = shape
        solid_primitive.dimensions.extend(dimensions)
        self.co.primitives.append(solid_primitive)

        primitive_pose = Pose()
        primitive_pose.position.x = offset[0]
        primitive_pose.position.y = offset[1]
        primitive_pose.position.z = offset[2]
        self.co.primitive_poses.append(primitive_pose)

        self.co.operation = CollisionObject.ADD

class World:
    def __init__(self):
        rospy.init_node('world_node')

        self.model_states = ModelStates()
        self.initialize_scene = False
        self.object_dict: Dict[str, CollisionObjectBuilder] = {}
        self.surface_dict: Dict[str, CollisionObjectBuilder] = {}
        
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

        self.planning_scene_service = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)

        rospy.wait_for_service('apply_planning_scene')
        rospy.Service('update_planning_scene', Trigger, self.update_planning_scene_request)

    def update_planning_scene(self):
        planning_scene = PlanningScene()
        for i, name in enumerate(self.model_states.name):
            co_builder = self.object_dict.get(name)
            if co_builder:
                co = co_builder.co
                co.pose = self.model_states.pose[i]
                co.operation = CollisionObject.MOVE
                planning_scene.world.collision_objects.append(co)
        planning_scene.is_diff = True
        self.planning_scene_service(planning_scene)

    def update_planning_scene_request(self, req):
        self.update_planning_scene()
        response = TriggerResponse(success=True, message="Planning scene updated")
        return response

    def initialize_planning_scene(self):
        planning_scene = PlanningScene()
        for i, name in enumerate(self.model_states.name):
            co_builder = self.object_dict.get(name) or self.surface_dict.get(name)
            if co_builder:
                co = co_builder.co
                co.pose = self.model_states.pose[i]
                planning_scene.world.collision_objects.append(co)

        for name in self.surface_dict:
            color = ObjectColor()
            color.id = name
            color.color.b = 1.0
            color.color.a = 1.0
            planning_scene.object_colors.append(color)

        planning_scene.is_diff = True
        self.planning_scene_service(planning_scene)
        print("Planning Scene Initialized")

    def model_states_callback(self, msg):
        self.model_states = msg

        if self.initialize_scene:
            print("Initializing Planning Scene")
            self.initialize_planning_scene()
            self.initialize_scene = False

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
                    surface = item.get('surface', False)
                    reference_frame = item['reference_frame']
                    shape = item['shape']
                    dimensions = item['dimensions']
                    offset = item['offset']

                    # Add object to dictionary
                    co_builder = CollisionObjectBuilder(name, reference_frame, shape, dimensions, offset)
                    if surface:
                        self.surface_dict[name] = co_builder
                    else:
                        self.object_dict[name] = co_builder

                self.initialize_scene = True

        except Exception as e:
            print(f"Error reading YAML file: {e}")
            return None

if __name__ == '__main__':
    world = World()
    
    file_path = rospkg.RosPack().get_path('world') + '/config/objects.yaml'
    world.read_yaml_file(file_path)
    rospy.spin()
