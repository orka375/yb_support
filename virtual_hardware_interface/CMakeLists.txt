cmake_minimum_required(VERSION 3.20)
project(virtual_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright/linter checks if desired
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Build the hardware interface library
add_library(${PROJECT_NAME} SHARED
  src/virtual_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  controller_interface
  geometry_msgs
  nav_msgs
  rclcpp_lifecycle
  pluginlib
)

# Export the plugin XML for ros2_control
pluginlib_export_plugin_description_file(hardware_interface virtual_hardware_interface.xml)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install plugin XML
install(
  FILES virtual_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
