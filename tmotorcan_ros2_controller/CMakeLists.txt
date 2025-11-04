cmake_minimum_required(VERSION 3.5)
project(tmotorcan_ros2_controller)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
