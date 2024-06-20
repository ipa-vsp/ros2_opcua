cmake_minimum_required(VERSION 3.8)
project(opcua_client_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(open62541pp REQUIRED)
find_package(open62541pp_vendor REQUIRED)

set(dependencies
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
    rclcpp_components
    yaml_cpp_vendor
    open62541pp
    open62541pp_vendor)

generate_datatype_headers(datatypes)

# Node OPC-UA Client
add_library(node_opcua_client SHARED src/node_interface/node_opcua_client.cpp)
target_include_directories(
  node_opcua_client
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(node_opcua_client ${dependencies})

# OPC-UA Client Manager Component
add_library(opcua_client_manager SHARED src/opcua_client_manager.cpp)
target_include_directories(
  opcua_client_manager
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(opcua_client_manager ${dependencies})
target_link_libraries(opcua_client_manager node_opcua_client)

rclcpp_components_register_nodes(opcua_client_manager
                                 "ros2_opcua::LifecycleOpcUAClient")

# Device Container
add_library(device_container SHARED src/device_container.cpp)
target_include_directories(
  device_container PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:include>)
ament_target_dependencies(device_container ${dependencies})
target_link_libraries(device_container node_opcua_client opcua_client_manager)

add_executable(device_container_node src/device_container_node.cpp)
target_include_directories(
  device_container_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(device_container_node ${dependencies})
target_link_libraries(device_container_node device_container node_opcua_client
                      opcua_client_manager)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS node_opcua_client opcua_client_manager device_container
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)

install(TARGETS device_container_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_export_libraries(node_opcua_client opcua_client_manager)

ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(${dependencies})

ament_package()
