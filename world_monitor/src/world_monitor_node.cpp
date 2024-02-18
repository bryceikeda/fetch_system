#include <world_monitor/world_monitor.h>

int 
main(int argc, char* argv[])
{
    ros::init( argc, argv, "world_monitor_node" );
	ros::NodeHandle nh;

	WorldMonitor world_monitor;
    
    world_monitor.object_detections_sub = nh.subscribe("/perception/object_detections", 1, &WorldMonitor::objectDetectionsCallback, &world_monitor);
    world_monitor.vision_info_sub = nh.subscribe("/perception/vision_info", 1, &WorldMonitor::visionInfoCallback, &world_monitor);
    world_monitor.surfaces_sub = nh.subscribe("/perception/surfaces", 1, &WorldMonitor::surfacesCallback, &world_monitor);
    world_monitor.planning_scene_service = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    world_monitor.update_planning_scene_service = nh.advertiseService("update_planning_scene", &WorldMonitor::updatePlanningSceneRequest, &world_monitor);
    
    bool initialize_scene = true; 

    ros::Rate loop_rate(40); 
	while(ros::ok()){
        if (initialize_scene) {
            initialize_scene = world_monitor.initializePlanningScene();
        }	
        ros::spinOnce();
		loop_rate.sleep(); 
	}
    return 0;  
}
