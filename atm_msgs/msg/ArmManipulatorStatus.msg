# Periodic Msg for '/arm_manipulator/status' topic for ATM by arm manipulator
# Provide status update to ATM or fleet manager

# Unique Manipulator ID
string manipulator_id

# Unique Task ID
string task_id

# Desired Arm Manipulator Position
## Pose ID are predefined position, defined in .config file.
##  'Nan': Undefined Position
##  'P1': Rest position
##  'P2': Cup Rack
##  'P3': Beverage Machine
##  'P4': Left Payload compartment
##  'P5': Right Payload compartment
string arm_pose_id

# End Effector State
## -1: error/undefined state
##  0: Open
##  1: closed
int8 eef_state

# is process done: True False
bool ready

# is there error?
bool error_flag
