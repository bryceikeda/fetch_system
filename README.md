# Fetch Robot System Overview
Below are descriptions of the different components that interconnect with the Fetch robot.
If build fails, build twice. 

## Executive
Runs the logic that runs everything together at once.

You can either run fake_joints_startup.launch to run without physics. Or you can run gazebo_startup.launch to run with physics. 

To simulate a task call, run the executive_node.py file 

## Manipulation
This is where code is run to interface with moveit_task_constructor. There are actions for picking, placing, complete pick and place, tucking the robot's arm and moving the robot's arm to the ready position, pouring and wiping. 

## Perception
This is where and Object Detection algorithm will idealy live such as Yolo. Right now, objects are detected from the /gazebo/model_states topic. When gazebo is running, this is automatically published.

If gazebo is not running, then run the model_states_publisher.py in the simulator. See Simulator section for more details. 

*Important* config/detectable_objects.yaml: To choose the objects to be percieved, add their information here 

## Simulator
This is where any simulation data is run. For example, you may launch either a fetch_gazebo simulation environment, or a fake_fetch.launch file that launches fetch with fake joint control similar to the Moveit Task Constructor demos. 

If gazebo is not running, then run the model_states_publisher.py to publish the /gazebo/model_states topic with fake objects.

*Important* config/model_states.yaml: To provide static non-gazebo object positions for testing with just rviz, add them here. They wont be displayed unless the perception config/detectable_objects.yaml file is updated to include them.

## World Monitor
This folder keeps track of the planning scene for moveit. It takes in the objects and publishes them to the planning scene to be used in the planners.  

*Important* config/object_definition.yaml: This file contains the collision object properties of each object and must be defined. 

## Scene Graph
This folder keeps track of the relationships between objects. Right now, if objects are on other objects it will provide a object -> supports -> other object relationship. It also provides attributes for place pose subframes. This is necessary for the manipulation pipeline


