cmake_minimum_required(VERSION 3.5)
project(dynamixel_gripper_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ----- ros2 custom msg -----

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GripState.msg"
  DEPENDENCIES builtin_interfaces
)

# ----- setup python scripts -----

ament_python_install_package(scripts/)

install(PROGRAMS
  scripts/dynamixel_gripper.py
  DESTINATION bin
)

ament_package()
