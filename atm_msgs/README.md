# atm_msgs
Custom-built ROS2 msgs for rostopics communication between atm and other controllers

**Ros2 Packages which are using `atm_msgs`:**
- delivery_compartment
- robot_manipulator_manager
- sesto_fm
- beverage_machine


## Setup

Build package...
```
cd /rmf/build/ros2/src
git clone git@github.com:RMFHOPE/atm_msgs.git
cd ~/rmf/build/ros2
colcon build --symlink-install --packages-select atm_msgs
source ~/rmf/build/ros2/install/local_setup.bash
```
