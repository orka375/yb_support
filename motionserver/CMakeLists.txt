cmake_minimum_required(VERSION 3.10)
project(motionserver)

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_msgs REQUIRED)

find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(my_msgs REQUIRED)


# ------------------------------------------------------------------------------
# Motion Action Node (C++)
# ------------------------------------------------------------------------------
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(moveit_motion_node src/moveit_motion_node.cpp)

# Link ROS 2 interface and system dependencies
ament_target_dependencies(moveit_motion_node
    rclcpp
    rclcpp_action
    moveit_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    moveit_core
    moveit_ros_planning_interface
    my_msgs
)

# Link non-ROS libraries
target_link_libraries(moveit_motion_node
    moveit_ros_planning_interface::moveit_move_group_interface
)




# # ------------------------------------------------------------------------------
# # MoveIt Launch (C++)
# # ------------------------------------------------------------------------------
# add_executable(moveit_launch src/moveit_launch.cpp)
# ament_target_dependencies(moveit_launch
#   rclcpp
#   ament_index_cpp
# )

# ------------------------------------------------------------------------------
# Install targets
# ------------------------------------------------------------------------------
install(TARGETS
  moveit_motion_node
  # moveit_launch
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Install launch files
# ------------------------------------------------------------------------------
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Testing
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ------------------------------------------------------------------------------
# Finalize
# ------------------------------------------------------------------------------
ament_package()
