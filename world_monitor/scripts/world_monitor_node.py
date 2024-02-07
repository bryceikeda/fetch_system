#!/usr/bin/env python3

import rospy
import rospkg
import time
from moveit_msgs.msg import PlanningScene, CollisionObject, ObjectColor
from world_monitor.msg import CollisionObjects
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Trigger, TriggerResponse
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose, BoundingBox3D, VisionInfo
from typing import Dict

class CollisionObjectBuilder:
    def __init__(self, id, frame_id, shape, pose, dimensions):
        self.co = CollisionObject()
        self.co.header.frame_id = frame_id
        self.co.id = id
        self.co.pose = pose
        solid_primitive = SolidPrimitive()
        solid_primitive.type = shape
        solid_primitive.dimensions.extend(dimensions)
        self.co.primitives.append(solid_primitive)
        primitive_pose = Pose()
        primitive_pose.orientation.w = 1
        self.co.primitive_poses.append(primitive_pose)
        self.co.operation = CollisionObject.MOVE

class WorldMonitor:
    def __init__(self):
        rospy.init_node('world_monitor_node')

        self.object_detections = Detection3DArray()
        self.surfaces = CollisionObjects()
        self.object_names = []

        rospy.Subscriber('/perception/object_detections', Detection3DArray, self.object_detections_callback)
        rospy.Subscriber('/perception/vision_info', VisionInfo, self.vision_info_callback)
        rospy.Subscriber('/perception/surfaces', CollisionObjects, self.surfaces_callback)

        self.planning_scene_service = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)
        rospy.wait_for_service('apply_planning_scene')
        
        rospy.Service('update_planning_scene', Trigger, self.update_planning_scene_request)
    
    def surfaces_callback(self, msg):
        self.surfaces = msg

    def object_detections_callback(self, msg): 
        self.object_detections = msg

    def vision_info_callback(self, msg):
        if msg.method == "parameter server" and not self.object_names:
            self.object_names = rospy.get_param(msg.database_location)

    def update_planning_scene_request(self, req):
        planning_scene = self.update_planning_scene()
        self.planning_scene_service(planning_scene)
        response = TriggerResponse(success=True, message="Planning Scene Updated")
        return response

    def update_planning_scene(self):
        planning_scene = PlanningScene()
        for detection in self.object_detections.detections:
            name = self.object_names[detection.results[0].id]
            frame_id = detection.header.frame_id
            position = detection.bbox.center
            dimensions = [detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z]
            co_builder = CollisionObjectBuilder(name, frame_id, 1, position, dimensions) 
            co_builder.co.operation = CollisionObject.MOVE
            planning_scene.world.collision_objects.append(co_builder.co)

        planning_scene.is_diff = True
        return planning_scene()

    def initialize_planning_scene(self):
        if not self.object_detections.detections or not self.surfaces.objects or not self.object_names:
            return True

        planning_scene = PlanningScene()
        self.add_surfaces_to_scene(planning_scene)
        self.add_object_detections_to_scene(planning_scene)
        planning_scene.is_diff = True

        try:
            self.planning_scene_service(planning_scene)
            print("Planning Scene Initialized")
            return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return True

    def add_surfaces_to_scene(self, planning_scene):
        for surface in self.surfaces.objects:
            color = ObjectColor()
            color.id = surface.id
            color.color.b = 1.0
            color.color.a = 1.0
            surface.operation = CollisionObject.ADD
            planning_scene.object_colors.append(color)
            planning_scene.world.collision_objects.append(surface)

    def add_object_detections_to_scene(self, planning_scene):
        for detection in self.object_detections.detections:
            name = self.object_names[detection.results[0].id]
            frame_id = detection.header.frame_id
            position = detection.bbox.center
            dimensions = [detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z]
            co_builder = CollisionObjectBuilder(name, frame_id, 1, position, dimensions)
            co_builder.co.operation = CollisionObject.ADD
            planning_scene.world.collision_objects.append(co_builder.co)

if __name__ == '__main__':
    world_monitor = WorldMonitor()
    
    initialize_scene = True
   
    # Delay planning scene to robot can move to starting pose
    time.sleep(2)

    rate = rospy.Rate(1) 
    
    while not rospy.is_shutdown():
        if initialize_scene:
            initialize_scene = world_monitor.initialize_planning_scene()
        rate.sleep()
