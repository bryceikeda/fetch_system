#include <world_monitor/world_monitor.h>

WorldMonitor::WorldMonitor()
{
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
    if (msg->method == "parameter server" && objects_info.empty())
    {  
        std::map<std::string, bool> tempDict;  
        ros::param::get(msg->database_location, tempDict);
        
        for (const auto& pair : tempDict) {
            objects_info.push_back(pair); 
        }
    }
}

bool 
WorldMonitor::getStaticWorldRequest(moveit_msgs::GetPlanningScene::Request& req, moveit_msgs::GetPlanningScene::Response& res)
{
    if (!static_world.collision_objects.empty()) 
    {
        res.scene.world = static_world; 
        return true;
    } 
    else 
    {
        return false;
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
        if(objects_info[detection.results[0].id].second == true){
            moveit_msgs::CollisionObject co; 
            co.id = objects_info[detection.results[0].id].first;
            co.header = detection.header;
            co.pose = detection.bbox.center;
            co.operation = moveit_msgs::CollisionObject::MOVE;
            planning_scene.world.collision_objects.push_back(co);
        }
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
    if (object_detections.detections.empty() || static_world.collision_objects.empty() || objects_info.empty()) 
    {   
        return true;
    }
    moveit_msgs::PlanningScene planning_scene;
    for(auto detection : object_detections.detections)
    {
        if(objects_info[detection.results[0].id].second == false){
            addStaticObjectToScene(planning_scene, detection);
        }
        else{
            addManipulableObjectToScene(planning_scene, detection);
        } 
    }

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
WorldMonitor::addManipulableObjectTransform(const vision_msgs::Detection3D& detection)
{
    geometry_msgs::TransformStamped transform; 
    transform.header.frame_id = collisionObject.header.frame_id;
    transform.child_frame_id = collisionObject.id;
    transform.transform.translation.x = collisionObject.pose.position.x;  
    transform.transform.translation.y = collisionObject.pose.position.y;  
    transform.transform.translation.z = collisionObject.pose.position.z;
    transform.transform.rotation = collisionObject.pose.orientation; 
    transformStampedArray.push_back(transform); 

    for(int i = 0; i < collisionObject.subframe_names.size(); i++)
    {
        geometry_msgs::TransformStamped transform; 
        transform.header.frame_id = collisionObject.id;
        transform.child_frame_id = collisionObject.subframe_names[i];
        transform.transform.translation.x = collisionObject.subframe_poses[i].position.x;  
        transform.transform.translation.y = collisionObject.subframe_poses[i].position.y;  
        transform.transform.translation.z = collisionObject.subframe_poses[i].position.z;  
        transform.transform.rotation = collisionObject.subframe_poses[i].orientation; 
        transformStampedArray.push_back(transform); 
    }
}

void 
WorldMonitor::addStaticObjectTransform(const moveit_msgs::CollisionObject& collisionObject)
{
    geometry_msgs::TransformStamped transform; 
    transform.header.frame_id = collisionObject.header.frame_id;
    transform.child_frame_id = collisionObject.id;
    transform.transform.translation.x = collisionObject.pose.position.x;  
    transform.transform.translation.y = collisionObject.pose.position.y;  
    transform.transform.translation.z = collisionObject.pose.position.z;
    transform.transform.rotation = collisionObject.pose.orientation; 
    transformStampedArray.push_back(transform); 

    for(int i = 0; i < collisionObject.subframe_names.size(); i++)
    {
        geometry_msgs::TransformStamped transform; 
        transform.header.frame_id = collisionObject.id;
        transform.child_frame_id = collisionObject.subframe_names[i];
        transform.transform.translation.x = collisionObject.subframe_poses[i].position.x;  
        transform.transform.translation.y = collisionObject.subframe_poses[i].position.y;  
        transform.transform.translation.z = collisionObject.subframe_poses[i].position.z;  
        transform.transform.rotation = collisionObject.subframe_poses[i].orientation; 
        transformStampedArray.push_back(transform); 
    }
}

void 
WorldMonitor::broadcastTransforms()
{
    for(auto transform : transformStampedArray)
    {
        transform.header.stamp = ros::Time::now(); 
        tfb.sendTransform(transform); 
    }
}

void
WorldMonitor::addStaticObjectToScene(moveit_msgs::PlanningScene& planning_scene, vision_msgs::Detection3D& detection) 
{
    // Add support surfaces that are blue
    for (auto static_object : static_world.collision_objects) {
        if(static_object.id == objects_info[detection.results[0].id].first){ 
            moveit_msgs::ObjectColor color;
            color.id = static_object.id;
            color.color.b = 1.0;
            color.color.a = 1.0;
            static_object.pose = detection.bbox.center;
            static_object.operation = moveit_msgs::CollisionObject::ADD;
            planning_scene.object_colors.push_back(color);
            planning_scene.world.collision_objects.push_back(static_object);
            addStaticObjectTransform(static_object); 
            ROS_INFO_STREAM("[world_monitor_node] Adding support static object to scene: " << static_object.id);
            break; 
        }
    }
}

void 
WorldMonitor::addManipulableObjectToScene(moveit_msgs::PlanningScene& planning_scene, vision_msgs::Detection3D& detection) 
{
    std::string name = objects_info[detection.results[0].id].first;

    // Bottle and glass are both meshes        
    if (name == "bottle" || name == "glass"){
        planning_scene.world.collision_objects.push_back(getManipulableObjectMesh(name, detection));
    }
    else
    {
        // All other objects are boxes or cylinders
        planning_scene.world.collision_objects.push_back(getManipulableObjectBasicShape(name, detection));
    }
    
    

    ROS_INFO_STREAM("[world_monitor_node] Adding manipulable object to scene: " << name);
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

void
WorldMonitor::loadStaticObjects(std::string filepath)
{
    // Read configuration from YAML file
    YAML::Node config = YAML::LoadFile(filepath);

    // Iterate over each entry in the YAML file
    for (const auto& entry : config) {
        // Create a CollisionObject for each entry
        moveit_msgs::CollisionObject collisionObject;

            // Safety checks for required fields
        if (!entry["object_id"] || !entry["header_frame_id"] || !entry["primitive_types"] ||
            !entry["primitive_dimensions"] || !entry["primitive_poses"] || !entry["subframe_names"] || !entry["subframe_poses"]) {
            // Handle missing fields or incorrect YAML structure
            ROS_ERROR_STREAM("Invalid YAML structure in the file.");
            continue;
        }


        collisionObject.id = entry["object_id"].as<std::string>();
        collisionObject.header.frame_id = entry["header_frame_id"].as<std::string>();

        // Get information arrays from the entry
        const auto& primitiveTypes = entry["primitive_types"];
        const auto& primitiveDimensions = entry["primitive_dimensions"];
        const auto& primitivePoses = entry["primitive_poses"];
       
        const auto& subframeNames = entry["subframe_names"].as<std::vector<std::string>>();
        const auto& subframePoses = entry["subframe_poses"];

        // Iterate over each primitive in the entry
        for (std::size_t i = 0; i < primitiveTypes.size(); ++i) {
            // Assuming there is only one type and pose per primitive
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitiveTypes[i][0].as<int>();
            primitive.dimensions = primitiveDimensions[i].as<std::vector<double>>();

            geometry_msgs::Pose primitivePose;
            primitivePose.position.x = primitivePoses[i][0].as<double>();
            primitivePose.position.y = primitivePoses[i][1].as<double>();
            primitivePose.position.z = primitivePoses[i][2].as<double>();
            primitivePose.orientation.x = primitivePoses[i][3].as<double>();
            primitivePose.orientation.y = primitivePoses[i][4].as<double>();
            primitivePose.orientation.z = primitivePoses[i][5].as<double>();
            primitivePose.orientation.w = primitivePoses[i][6].as<double>(); 
            // Set the pose and primitive for the CollisionObject
            collisionObject.primitives.push_back(primitive);
            collisionObject.primitive_poses.push_back(primitivePose);
            
        }
        
        for (std::size_t i = 0; i < subframeNames.size(); ++i) {
            geometry_msgs::Pose subframePose;
            subframePose.position.x = subframePoses[i][0].as<double>();
            subframePose.position.y = subframePoses[i][1].as<double>();
            subframePose.position.z = subframePoses[i][2].as<double>();
            subframePose.orientation.x = subframePoses[i][3].as<double>();
            subframePose.orientation.y = subframePoses[i][4].as<double>();
            subframePose.orientation.z = subframePoses[i][5].as<double>();
            subframePose.orientation.w = subframePoses[i][6].as<double>(); 
            // Set the pose and primitive for the CollisionObject
            collisionObject.subframe_names.push_back(subframeNames[i]);
            collisionObject.subframe_poses.push_back(subframePose);
        }
        // Add the CollisionObject to the list
        static_world.collision_objects.push_back(collisionObject);
    }
}
