#ifndef WORLD_MONITOR_H
#define WORLD_MONITOR_H

#include <ros/ros.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningSceneWorld.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/VisionInfo.h>
#include <shape_msgs/SolidPrimitive.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_extents.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/TransformStamped.h>
#include <shape_msgs/Mesh.h>
#include <yaml-cpp/yaml.h>

class WorldMonitor
{
public: 
    WorldMonitor();
    ~WorldMonitor() = default; 
   
    // PSI
    bool initializePlanningScene();
    bool updatePlanningSceneRequest(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool getStaticWorldRequest(moveit_msgs::GetPlanningScene::Request& req, moveit_msgs::GetPlanningScene::Response& res);
    bool applyPlanningScene(moveit_msgs::PlanningScene& planning_scene);
    void addStaticObjectToScene(moveit_msgs::PlanningScene& planning_scene, vision_msgs::Detection3D& detection);
    void addManipulableObjectToScene(moveit_msgs::PlanningScene& planning_scene, vision_msgs::Detection3D& detection);
    void loadStaticObjects(const std::string filepath);

    moveit_msgs::PlanningScene updatePlanningScene();
    moveit_msgs::CollisionObject getManipulableObjectBasicShape(const std::string& name, const vision_msgs::Detection3D& detection); 
    moveit_msgs::CollisionObject getManipulableObjectMesh(const std::string& name, const vision_msgs::Detection3D& detection); 
    
    // Object detection/tracking
    void objectDetectionsCallback(const vision_msgs::Detection3DArray::ConstPtr& msg);
    void visionInfoCallback(const vision_msgs::VisionInfo::ConstPtr& msg);
    double computeMeshHeight(const shape_msgs::Mesh &mesh);
    void addStaticObjectTransform(const moveit_msgs::CollisionObject& collisionObject); 
    void addManipulableObjectTransform(const moveit_msgs::CollisionObject& collisionObject); 
    void broadcastTransforms();

    // ROS Communication
    ros::Subscriber object_detections_sub;
    ros::Subscriber vision_info_sub;
    ros::ServiceClient planning_scene_service;
    ros::ServiceServer update_planning_scene_service;
    ros::ServiceServer get_static_world_service;
    tf2_ros::TransformBroadcaster tfb; 

private:
    vision_msgs::Detection3DArray object_detections;
    moveit_msgs::PlanningSceneWorld static_world;
    std::vector<std::pair<std::string, bool>> objects_info;
    std::vector<geometry_msgs::TransformStamped> transformStampedArray; 
};

#endif
