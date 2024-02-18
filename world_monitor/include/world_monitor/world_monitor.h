#ifndef WORLD_MONITOR_H
#define WORLD_MONITOR_H

#include <ros/ros.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_srvs/Trigger.h>
#include <world_monitor/CollisionObjects.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/VisionInfo.h>
#include <shape_msgs/SolidPrimitive.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_extents.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>

class WorldMonitor
{
public: 
    WorldMonitor();
    ~WorldMonitor() = default; 
    
    void surfacesCallback(const world_monitor::CollisionObjects::ConstPtr& msg);
    void objectDetectionsCallback(const vision_msgs::Detection3DArray::ConstPtr& msg);
    void visionInfoCallback(const vision_msgs::VisionInfo::ConstPtr& msg);
    bool updatePlanningSceneRequest(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool applyPlanningScene(moveit_msgs::PlanningScene& planning_scene);
    moveit_msgs::PlanningScene updatePlanningScene();
    bool initializePlanningScene();
    void addSurfacesToScene(moveit_msgs::PlanningScene& planning_scene);
    void addObjectDetectionsToScene(moveit_msgs::PlanningScene& planning_scene);
    moveit_msgs::CollisionObject getManipulableObjectBasicShape(const std::string& name, const vision_msgs::Detection3D& detection); 
    moveit_msgs::CollisionObject getManipulableObjectMesh(const std::string& name, const vision_msgs::Detection3D& detection); 
    double computeMeshHeight(const shape_msgs::Mesh &mesh);
    
    ros::Subscriber object_detections_sub;
    ros::Subscriber vision_info_sub;
    ros::Subscriber surfaces_sub;
    ros::ServiceClient planning_scene_service;
    ros::ServiceServer update_planning_scene_service;

private:
    vision_msgs::Detection3DArray object_detections;
    world_monitor::CollisionObjects surfaces;
    std::vector<std::string> object_names;
};



#endif
