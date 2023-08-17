cmake_minimum_required(VERSION 3.8)
project(opcua_client_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Set the path to the open62541pp submodule
set(open62541pp_DIR "${CMAKE_CURRENT_SOURCE_DIR}/open62541pp")

# Set the CMake arguments for the library
set(UAPP_BUILD_EXAMPLES ON CACHE BOOL "Build examples for open62541pp")
set(UAPP_BUILD_TESTS ON CACHE BOOL "Build tests for open62541pp")
set(CMAKE_BUILD_TYPE Debug CACHE STRING "Set the build type")

# Add the open62541pp submodule to the build
add_subdirectory(${open62541pp_DIR} open62541pp_build)

# If you're creating a node, for instance, "example_node"
add_executable(example_node src/example_node.cpp)
ament_target_dependencies(example_node rclcpp)

# Linking the open62541pp library to the node
target_link_libraries(example_node open62541pp)

# Installation
install(TARGETS example_node
  DESTINATION lib/${PROJECT_NAME}
)

# Here, we assume that the library is being built in the submodule's CMake.
# Make sure the name matches (case-sensitive).
install(TARGETS open62541pp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export the necessary directories
ament_export_include_directories(include)
ament_export_libraries(open62541pp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
