#!/usr/bin/env python3

import yaml
import rospy
import rospkg
from gazebo_msgs.msg import ModelStates
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose, BoundingBox3D, VisionInfo
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from world_monitor.msg import CollisionObjects

class DetectedObjectBuilder:
    def __init__(self, id, frame_id, size):
        self.detection = Detection3D()
        self.detection.header.frame_id = frame_id
        result = ObjectHypothesisWithPose()
        result.id = id
        self.detection.results.append(result)
        self.detection.bbox.center = Pose()
        self.detection.bbox.center.orientation.w = 1
        self.detection.bbox.size.x = size[0]
        self.detection.bbox.size.y = size[1]
        self.detection.bbox.size.z = size[2]

class SurfaceBuilder:
    def __init__(self, name, frame_id, shape, dimensions, offset):
        self.co = CollisionObject()
        self.co.header.frame_id = frame_id
        self.co.id = name

        solid_primitive = SolidPrimitive()
        solid_primitive.type = shape
        solid_primitive.dimensions.extend(dimensions)
        self.co.primitives.append(solid_primitive)

        primitive_pose = Pose()
        primitive_pose.position.x = offset[0]
        primitive_pose.position.y = offset[1]
        primitive_pose.position.z = offset[2]
        primitive_pose.orientation.w = 1
        self.co.primitive_poses.append(primitive_pose)

        self.co.operation = CollisionObject.MOVE

class Perception:
    def __init__(self):
        rospy.init_node('gazebo_perception_node')

        self.model_states = ModelStates()
        self.object_dict = {}
        self.surface_dict = {}

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

        self.vision_info_pub = rospy.Publisher('/perception/vision_info', VisionInfo, queue_size=1)   
        self.surfaces_pub = rospy.Publisher('/perception/surfaces', CollisionObjects, queue_size=1)   
        self.object_detections_pub = rospy.Publisher('/perception/object_detections', Detection3DArray, queue_size=1)   
    def model_states_callback(self, msg):
        self.model_states = msg 

    def object_detections_publisher(self):
        detections = Detection3DArray()
        
        for i, name in enumerate(self.model_states.name):
            gazebo_object = self.object_dict.get(name)
            if gazebo_object:
                gazebo_object.detection.bbox.center = self.model_states.pose[i]
                detections.detections.append(gazebo_object.detection)
            
        self.object_detections_pub.publish(detections)

    def vision_info_publisher(self):
        vision_info = VisionInfo()
        vision_info.method = "parameter server"
        vision_info.database_location = "/vision_info_lookup"
        self.vision_info_pub.publish(vision_info)

    def surfaces_publisher(self):
        surfaces = CollisionObjects()
        
        for i, name in enumerate(self.model_states.name):
            surface = self.surface_dict.get(name)
            if surface:
                surface = surface.co
                surface.pose = self.model_states.pose[i]
                surfaces.objects.append(surface)
        self.surfaces_pub.publish(surfaces) 

    def read_objects_yaml_file(self, file_path):
        try:
            # Open the YAML file for reading
            with open(file_path, 'r') as file:
                # Load the YAML data
                yaml_data = yaml.safe_load(file)
                
                object_list = []

                # Iterate through each object in the YAML data
                for item in yaml_data:
                    # Extract relevant attributes
                    name = item['name']
                    id = item['id']
                    frame_id = item['frame_id']
                    size = item['size']

                    # Add object to dictionary
                    object_builder = DetectedObjectBuilder(id, frame_id, size)
                    self.object_dict[name] = object_builder

                    object_list.append(name)

                rospy.set_param('/vision_info_lookup', object_list)

        except Exception as e:
            print(f"Error reading YAML file: {e}")
            return None

    def read_surfaces_yaml_file(self, file_path):
        try:
            # Open the YAML file for reading
            with open(file_path, 'r') as file:
                # Load the YAML data
                yaml_data = yaml.safe_load(file)
                
                # Iterate through each object in the YAML data
                for item in yaml_data:
                    # Extract relevant attributes
                    name = item['name']
                    frame_id = item['frame_id']
                    shape = item['shape']
                    dimensions = item['dimensions']
                    offset = item['offset']

                    # Add object to dictionary
                    surface_builder = SurfaceBuilder(name, frame_id, shape, dimensions, offset)
                    self.surface_dict[name] = surface_builder

        except Exception as e:
            print(f"Error reading YAML file: {e}")
            return None

if __name__ == '__main__':
    perception_node = Perception()
    
    objects_relative_path = rospy.get_param("~objects_relative_path", "/config/manipulable_objects.yaml")
    surfaces_relative_path = rospy.get_param("~surfaces_relative_path", "/config/support_surfaces.yaml")
    
    objects_file_path = rospkg.RosPack().get_path('perception') + objects_relative_path
    perception_node.read_objects_yaml_file(objects_file_path)
   
    surfaces_file_path = rospkg.RosPack().get_path('perception') + surfaces_relative_path
    perception_node.read_surfaces_yaml_file(surfaces_file_path)
   

    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        perception_node.object_detections_publisher()
        perception_node.surfaces_publisher()
        perception_node.vision_info_publisher()
        rate.sleep()
