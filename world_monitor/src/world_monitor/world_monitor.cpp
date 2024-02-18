#include <world_monitor/world_monitor.h>

WorldMonitor::WorldMonitor()
{
}

void 
WorldMonitor::surfacesCallback(const world_monitor::CollisionObjects::ConstPtr& msg)
{
    surfaces = *msg;
}

void 
WorldMonitor::objectDetectionsCallback(const vision_msgs::Detection3DArray::ConstPtr& msg)
{
    object_detections = *msg;
}

void
WorldMonitor::visionInfoCallback(const vision_msgs::VisionInfo::ConstPtr& msg)
{
    // Assuming all manipulable objects are known, add them to object_names array
    if (msg->method == "parameter server" && object_names.empty())
    {   
        ros::param::get(msg->database_location, object_names);
    }
}

bool 
WorldMonitor::updatePlanningSceneRequest(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    // Service call /update_planning_scene will move object positions
    moveit_msgs::PlanningScene planning_scene = updatePlanningScene();
    
    if (applyPlanningScene(planning_scene)) 
    {
        res.success = true;
        res.message = "Planning Scene Updated";
        return true;
    } 
    else 
    {
        res.success = false;
        res.message = "Failed to update planning scene";
        return false;
    }
}

moveit_msgs::PlanningScene 
WorldMonitor::updatePlanningScene()
{
    // Update planning scene based on what is viewed
    moveit_msgs::PlanningScene planning_scene;

    for (const auto& detection : object_detections.detections) {
        moveit_msgs::CollisionObject co; 
        co.id = object_names[detection.results[0].id];
        co.header = detection.header;
        co.pose = detection.bbox.center;
        co.operation = moveit_msgs::CollisionObject::MOVE;
        planning_scene.world.collision_objects.push_back(co);
    }
    planning_scene.is_diff = true;
    return planning_scene;
}

bool
WorldMonitor::applyPlanningScene(moveit_msgs::PlanningScene& planning_scene)
{
    // Apply Planning scene for initial add and continued update
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    if (planning_scene_service.call(srv)) 
    {
        ROS_INFO("[world_monitor_node] Planning Scene applied.");
        return true;
    } 
    else 
    {
        ROS_ERROR_STREAM("[world_monitor_node] Failed to call Planning Scene service");
        return false;
    }
}

bool
WorldMonitor::initializePlanningScene()
{
    // If there are no detections, then don't initialize the planning scene
    if (object_detections.detections.empty() || surfaces.objects.empty() || object_names.empty()) 
    {   
        return true;
    }
    moveit_msgs::PlanningScene planning_scene;

    // Add support surfaces that don't move 
    addSurfacesToScene(planning_scene);

    // Add manipulable objects that get added to list. Can move.
    addObjectDetectionsToScene(planning_scene);

    planning_scene.is_diff = true;

    if (applyPlanningScene(planning_scene))
    {
        ROS_INFO("[world_monitor_node] Planning Scene Initialized");
        return false; 
    }
    else 
    {
        ROS_ERROR_STREAM("[world_monitor_node] Service call failed");
        return true;
    }
}

void
WorldMonitor::addSurfacesToScene(moveit_msgs::PlanningScene& planning_scene) 
{
    // Add support surfaces that are blue
    for (auto& surface : surfaces.objects) {
        moveit_msgs::ObjectColor color;
        color.id = surface.id;
        color.color.b = 1.0;
        color.color.a = 1.0;
        surface.operation = moveit_msgs::CollisionObject::ADD;
        planning_scene.object_colors.push_back(color);
        planning_scene.world.collision_objects.push_back(surface);
        ROS_INFO_STREAM("[world_monitor_node] Adding support surface to scene: " << surface.id);
    }
}

void 
WorldMonitor::addObjectDetectionsToScene(moveit_msgs::PlanningScene& planning_scene) 
{
    for (const auto& detection : object_detections.detections) {
        std::string name = object_names[detection.results[0].id];

        // Bottle and glass are both meshes        
        if (name == "bottle" || name == "glass"){
            planning_scene.world.collision_objects.push_back(getManipulableObjectMesh(name, detection));
        }
        else
        {
            // All other objects are boxes
            planning_scene.world.collision_objects.push_back(getManipulableObjectBasicShape(name, detection));
        }

        ROS_INFO_STREAM("[world_monitor_node] Adding manipulable object to scene: " << name);
    }
}

moveit_msgs::CollisionObject
WorldMonitor::getManipulableObjectBasicShape(const std::string& name, const vision_msgs::Detection3D& detection)
{
    // Load just a box primitive because bounding boxes are just that. 
    moveit_msgs::CollisionObject co;
    co.header.frame_id = detection.header.frame_id;
    co.id = name;  
    co.pose = detection.bbox.center;

    shape_msgs::SolidPrimitive solid_primitive;
    
    // Could find a way to make this cylinder if necessary
    if (name == "cylinder"){
        solid_primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
        solid_primitive.dimensions = {detection.bbox.size.z, detection.bbox.size.x/2}; 
    }
    else{
        solid_primitive.type = shape_msgs::SolidPrimitive::BOX; 
        solid_primitive.dimensions = {detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z};
    }

    co.primitives.push_back(solid_primitive);

    geometry_msgs::Pose primitive_pose;
    primitive_pose.orientation.w = 1;
    co.primitive_poses.push_back(primitive_pose);
    
    return co; 
}

moveit_msgs::CollisionObject
WorldMonitor::getManipulableObjectMesh(const std::string& name, const vision_msgs::Detection3D& detection)
{
    // Load meshes of objects into the scene
    moveit_msgs::CollisionObject co;
    co.id = name; 
    co.header = detection.header;
    co.pose = detection.bbox.center;

    std::string resource = "package://world_monitor/meshes/" + name + ".stl";

    // load mesh
    const Eigen::Vector3d scaling(1, 1, 1);
    shapes::Shape *shape = shapes::createMeshFromResource(resource, scaling);
    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(shape, shape_msg);
 
    co.meshes.resize(1);
    co.meshes[0] = boost::get<shape_msgs::Mesh>(shape_msg);

    co.mesh_poses.resize(1);
    co.mesh_poses[0].orientation.w = 1.0;
    co.mesh_poses[0].position.z += computeMeshHeight(co.meshes[0]) / 2 + 0.002;

    co.operation = moveit_msgs::CollisionObject::ADD;
    
    return co; 
}

double 
WorldMonitor::computeMeshHeight(const shape_msgs::Mesh &mesh) {
  double x, y, z;
  geometric_shapes::getShapeExtents(mesh, x, y, z);
  return z;
}
