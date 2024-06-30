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

# Define the dependencies in a set
set(DEPENDENCIES
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
      test/main.cpp
      test/test_waypoint_action_server.cpp
      )
  # Add a gtest executables
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  # Link the dependencies to the test executable
  ament_target_dependencies(${PROJECT_NAME}_test ${DEPENDENCIES})
  
  rosidl_target_interfaces(${PROJECT_NAME}_test
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )

  #target_link_libraries(${PROJECT_NAME}_test distance_control_lib)

  install(TARGETS
        ${PROJECT_NAME}_test
        # Install the generated test executable file to the path after DESTINATION
        DESTINATION lib/${PROJECT_NAME})
  # uncomment to enforce lintners and static code analyzers defined in ament_lint_common package        
#   ament_lint_auto_find_test_dependencies()
endif()


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
