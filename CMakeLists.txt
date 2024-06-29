cmake_minimum_required(VERSION 3.5)
project(tortoisebot_waypoints)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/WaypointAction.action"
  DEPENDENCIES geometry_msgs
)

add_executable(tortoisebot_action_server src/tortoisebot_action_server.cpp)

ament_target_dependencies(tortoisebot_action_server
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

rosidl_target_interfaces(tortoisebot_action_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS
  tortoisebot_action_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
