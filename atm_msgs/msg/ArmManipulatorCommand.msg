# Msg for '/arm_manipulator/command' topic for arm manipulator by ATM
# Command arm manipulator to execute certain defined action

# Unique Manipulator ID
string manipulator_id

# Unique Task ID
string task_id

# Desired Arm Manipulator Position
## Pose ID are predefined position, defined in .config file.
##  'P1': Rest position
##  'P2': Cup Rack
##  'P3': Beverage Machine
##  'P4': Left Payload compartment
##  'P5': Right Payload compartment
##  'err': error/undefined pos
string arm_pose_id

# End Effector Manipulator State
##  0: Open
##  1: closed
## -1: error/undefined state
int8 eef_command


# Type of drink to dispense at beverage_machine pose
# 1: hotwater, 2: Kopi latte, 3: teh latte, 4: choco Matt
int8 drink_select
