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

include(FetchContent)

FetchContent_Declare(
  open62541pp
  GIT_REPOSITORY https://github.com/open62541pp/open62541pp.git
  GIT_TAG 1eda60d662c2afb73d0f15898f8eb98c6f85e956
  GIT_PROGRESS TRUE
  GIT_SHALLOW TRUE
)

# # # Set the CMake arguments for the library
set(UAPP_BUILD_EXAMPLES ON CACHE BOOL "Build examples for open62541pp")
set(UAPP_BUILD_TESTS ON CACHE BOOL "Build tests for open62541pp")
set(CMAKE_BUILD_TYPE Debug CACHE STRING "Set the build type")

FetchContent_GetProperties(open62541pp)
if(NOT open62541pp_POPULATED)
  FetchContent_Populate(open62541pp)
  add_subdirectory(${open62541pp_SOURCE_DIR} ${open62541pp_BINARY_DIR})
endif()

# message(${open62541pp_DIR})
# # If you're creating a node, for instance, "example_node"
add_executable(omros_client_node src/omros_client_node.cpp)
target_include_directories(omros_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(omros_client_node rclcpp)
# Linking the open62541pp library to the node
target_link_libraries(omros_client_node open62541pp)
# Installation
install(TARGETS omros_client_node
  DESTINATION lib/${PROJECT_NAME}
)


# Tutorial examples
add_executable(example_server_node src/example_server_node.cpp)
target_include_directories(example_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(example_server_node rclcpp)
# Linking the open62541pp library to the node
target_link_libraries(example_server_node open62541pp)
# Installation
install(TARGETS example_server_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(example_client_node src/example_client_node.cpp)
target_include_directories(example_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(example_client_node rclcpp)
# Linking the open62541pp library to the node
target_link_libraries(example_client_node open62541pp)
# Installation
install(TARGETS example_client_node
  DESTINATION lib/${PROJECT_NAME}
)

############################
install(DIRECTORY include/
  DESTINATION include/
)

# Here, we assume that the library is being built in the submodule's CMake.
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
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
