#!/usr/bin/env python3
import yaml
import rospy
import rospkg
from gazebo_msgs.msg import ModelStates
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose, BoundingBox3D
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion

class DetectedObjectBuilder:
    def __init__(self, id, frame_id, center, size):
        self.detection = Detection3D()
        self.detection.header.frame_id = frame_id
        result = ObjectHypothesisWithPose()
        result.id = 1
        self.detection.results.append(result)
        self.detection.bbox.center = center
        self.detection.bbox.size.x = size[0]
        self.detection.bbox.size.y = size[1]
        self.detection.bbox.size.z = size[2]

class Perception:
    def __init__(self):
        rospy.init_node('gazebo_perception_node')

        self.model_states = ModelStates()
        self.object_dict = {}

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

        self.detections_publisher = rospy.Publisher('/perception/object_detections', Detection3DArray, queue_size=1)   

    def model_states_callback(self, msg):
        self.model_states = msg 

    def object_detections_publisher(self):
        detections = Detection3DArray()
        
        for i, name in enumerate(self.model_states.name):
            gazebo_object = self.object_dict.get(name)
            if gazebo_object:
                gazebo_object.center = self.model_states.pose[i]
                detections.detections.append(gazebo_object.detection)
            
        self.detections_publisher.publish(detections)

    def read_yaml_file(self, file_path):
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
                    object_builder = DetectedObjectBuilder(id, frame_id, Pose(), size)
                    self.object_dict[name] = object_builder

                    object_list.append(name)

                rospy.set_param('/vision_info_lookup', object_list)

        except Exception as e:
            print(f"Error reading YAML file: {e}")
            return None

if __name__ == '__main__':
    perception_node = Perception()
    file_path = rospkg.RosPack().get_path('perception') + '/config/gazebo_objects.yaml'
    perception_node.read_yaml_file(file_path)
   
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        perception_node.object_detections_publisher()
        rate.sleep()
