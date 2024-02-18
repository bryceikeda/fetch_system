# By default, the task type is PICK_AND_PLACE
int32 PICK=0
int32 PLACE=1
int32 MOVE_ARM=2
int32 OPEN_GRIPPER=3
int32 CLOSE_GRIPPER=4
int32 WAVE=5
int32 DANCE=6
int32 WIPE=6
int32 POUR=7
int32 task_type

# Name to label the task
string task_name

# Name of object being picked
string object_name

# Place Surface
string[] support_surfaces

# Place Position
geometry_msgs/Pose place_pose


