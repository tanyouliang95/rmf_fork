# patient_device

# Getting  Started
This is a ros2 package operating on RaspberryPi 3B+. 
This package does the following:
1. When the patient triggers the push button (call button), the package notifies the `RCL Webserver` regarding the triggered button by the patient by sending the `device ID` from where the button action is triggered. We have an LED on the patient device to indicate different state of the call. It is at `ON` state when the patient triggers the call button.
2. ROS2 DDS is used to handle the communication between the raspberry pi and webserver. The call initiated notification is sent to the webserver via the ROS2 topic `/patient_device/call_button_state` with the message type `{deviceID: string, state: int8}`
3. The package sends `call_button_state` message every 2 seconds to the webserver.
4. Later, when the call has ended, the webserver will send the `deviceID` as notification to the patient device package via the ROS2 topic `/patient_device/call_button_action` and message type `{deviceID: string, state: int8}` indicating that it has successfully completed the call and ready for the next call initiation action by the patient

Few Topics are used to handle communication:
```
- /patient_device/call_button_state        :   {deviceID: string; status: int}
- /patient_device/call_button_action       :   {deviceID: string; status: int}
```

![alt text](https://github.com/RMFHOPE/patient_device/blob/master/documentation/PatientDevice.png)

# Build
```
cd rmf/build/ros2/
colcon build --symlink-install --packages-select patient_device --cmake-clean-cache
source ~/hope_msgs/ros2/hope_msgs/install/setup.bash
source install/setup.bash
ros2 run patient_device PatientDevice __params:=/home/pi/rmf/build/ros2/src/patient_device/params/parameters.yaml
```

Can run on ROS_DOMAIN_ID=20 if testing on different machines. Run the command
`export ROS_DOMAIN_ID=20` 
and source ROS workspace.

# Testing
```ros2 topic pub /patient_device/call_button_action hope_msgs/String "data: PD00x"```


## Automatic script running on bootup
Crontab is enabled on the patient device pi to start the ros2 node on bootup.
