# By default, the task type is PICK_AND_PLACE
int32 PICK_AND_PLACE=0
int32 PICK=1
int32 PLACE=2
int32 MOVE=3
int32 task_type

# Name of object being picked
string object_name

# Place Surface
string surface_name

# Place Position
geometry_msgs/Pose place_pose
