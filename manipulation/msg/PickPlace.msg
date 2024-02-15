# By default, the task type is PICK_AND_PLACE
int32 PICK_AND_PLACE=0
int32 PICK_ONLY=1
int32 PLACE_ONLY=2
int32 task_type

# Grasp Frame it will used
int32 HORIZONTAL=0
int32 VERTICAL=1
int32 DIAGONAL=2
int32 grasp_frame

# If execution or simulation
bool execute

# Name of object being picked
string object_name

# Place Position
geometry_msgs/Pose place_pose
