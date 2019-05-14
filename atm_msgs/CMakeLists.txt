cmake_minimum_required(VERSION 3.5)

project(atm_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(atm_msgs
  "msg/AgvCommand.msg"
  "msg/PayloadCommand.msg"
  "msg/PayloadStatus.msg"
  "msg/ArmManipulatorCommand.msg"
  "msg/ArmManipulatorStatus.msg"
  "msg/TaskInformation.msg"
  "msg/BeverageMachineCommand.msg"
  "msg/BeverageMachineStatus.msg"
  "msg/AlarmingEvent.msg"
  "msg/LiftRequest.msg"
  "msg/LiftState.msg"
  "msg/SestoFleetAdapterTask.msg"
  "msg/RobotState.msg"
  "msg/MapLocation.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
