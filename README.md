# Fetch Robot System Overview
Below are descriptions of the different components that interconnect with the Fetch robot.

## Executive
Runs the logic that runs everything together at once.

You can either run fake_joints_startup.launch to run without physics. Or you can run gazebo_startup.launch to run with physics. 

## Manipulation
This is where code is run to interface with moveit_task_constructor. There are actions for picking, placing, complete pick and place, tucking the robot's arm and moving the robot's arm to the ready position. 

## Perception
This is where and Object Detection algorithm will idealy live such as Yolo. Right now, objects are detected from the /gazebo/model_states topic. When gazebo is running, this is automatically published. To choose the objects to be percieved as manipulable, add the information to config/manipulable_objects.yaml. Similarly, update config/support_surfaces.yaml to define what tables are in the scene. 

If gazebo is not running, then run the model_states_publisher.py in the simulator. See Simulator section for more details. 

## Simulator
This is where any simulation data is run. For example, you may launch either a fetch_gazebo simulation environment, or a fake_fetch.launch file that launches fetch with fake joint control similar to the Moveit Task Constructor demos. 

If gazebo is not running, then run the model_states_publisher.py to publish the /gazebo/model_states topic with fake objects. 

The simulator can also spawn objects with the object_spawner.py. Read the README.md in the folder for more details. 

## World Monitor
This folder keeps track of the planning scene for moveit. It takes in the objects and publishes them to the planning scene to be used in the planners.  
