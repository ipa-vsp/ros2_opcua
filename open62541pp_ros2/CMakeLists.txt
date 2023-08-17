cmake_minimum_required(VERSION 3.5)
project(open62541pp_ros2)

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Set the path to the open62541pp submodule
set(open62541pp_DIR "${CMAKE_CURRENT_SOURCE_DIR}/open62541pp")

# Set the CMake arguments for the library
set(UAPP_BUILD_EXAMPLES ON CACHE BOOL "Build examples for open62541pp")
set(UAPP_BUILD_TESTS ON CACHE BOOL "Build tests for open62541pp")
set(CMAKE_BUILD_TYPE Debug CACHE STRING "Set the build type")

# Add the open62541pp submodule to the build
add_subdirectory(${open62541pp_DIR} open62541pp_build)

# # If you're creating a node, for instance, "example_node"
add_executable(example_node src/example_node.cpp)
ament_target_dependencies(example_node rclcpp)

# Linking the open62541pp library to the node
target_link_libraries(example_node open62541pp)

# Installation
install(TARGETS example_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS open62541pp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(open62541pp)

# Environment hooks, dependencies, etc.
ament_package()