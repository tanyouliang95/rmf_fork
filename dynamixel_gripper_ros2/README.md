# Dynamixel Servo Gripper [ROS2]

## Introduction

An end effector is the device at the end of a robotic arm, designed to interact with the environment. To provide for a low cost gripper solution for the underlying project, 2 [dynamixel motors](https://www.trossenrobotics.com/shared/images/PImages/R-903-0188-000-c.jpg) with claws affixed are attached alongside each other. This library (**built & tested with ROS2 bouncy, Ubuntu 16.04**) provides an easy-to-use ROS2 package to control the grip.

## Getting started

* Ensure dynamixel motors have the proper power supply & both servos have a different ID (you can change them by using a [GUI tool](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/#gui)
* Install [dynamixel_sdk](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/#repository)
* Serial Comm: `sudo pip3 install pyserial`
* Download & place these packages in $YOUR_ROS2_WS/src
* Source your ROS2 distro
* Run the following commands (root folder of your ROS2 workspace):

```
source $ROS2_WS/install/local_setup.bash
colcon build  --symlink-install --packages-select  dynamixel_gripper_ros2 --cmake-clean-cache
# if face stderr on build, try edit cmakelist, compile msg first then only python path
source ros2_ws/install/local_setup.bash
```

## Running the Gripper

Start and test!
```
sudo chmod 666 /dev/ttyUSB0
export ROS_DOMAIN_ID=0
dynamixel_gripper.py
ros2 topic pub /gripper/grip_command std_msgs/Bool "{data: True}" --once    # close
ros2 topic pub /gripper/grip_command std_msgs/Bool "{data: False}" --once   # open
```
This will initialise the gripper node, which you will the gripper opens and closes.

## ROS2 Topics

**Published Topics**
* /gripper/grip_state (custom msg type: dynamixel_gripper/GripState)
  - `is_gripped` returns `False` if gripper is open, and `True` if gripper is closed; contains other motor info as well
  - Returns the current load adn position for both servos

**Subscribed Topics**
* /gripper/grip_command (msg type: std_msgs/Bool)
  - `False` - Open
  - `True` - close

## Important Notes
* User can tune macro params in the python scripts for the gripper control
* Gripper may get hot after prolonged use. Keep track and ensure its **temperature does not exceed 65 degrees celsius**. To track temperature, check published topic **/gripper/grip_state**
* Ensure the dynamixel port in use follows _"/dev/ttyUSB0"_
* To give permenant autorization on `chmod 666` command, refer to this [link](https://unix.stackexchange.com/questions/25258/ttyusb0-permission-changes-after-restart)
* Left dynamixel gripper servo should be labeled as `ID 1`, and right dynamixel gripper servo labeled as `ID 2`

**Some Behavior of Dynamixel Motors**
- if certain motor position is given to the motor (via addr &30) and the position cant be reached due to overloading/obstacles, motor will disable it's stall torque.
- All low level behavior motor control can be tuned via the register addresses [here](http://support.robotis.com/en/product/actuator/dynamixel/ax_series/ax-18f.htm#)
- To understand the very details of the motor control, refer to the usage of `Dynamixel_SDK` [HERE](http://emanual.robotis.com/docs/en/dxl/protocol1/).
