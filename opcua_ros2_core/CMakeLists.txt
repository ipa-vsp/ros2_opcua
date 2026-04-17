cmake_minimum_required(VERSION 3.16)
project(opcua_ros2_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------------------------
# Dependencies
# -------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(opcua_ros2_vendor REQUIRED)
find_package(opcua_ros2_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

# open62541pp is installed by the vendor package into a cmake prefix path
# entry appended by opcua_ros2_vendor's cmake extras.
find_package(open62541pp REQUIRED)

# -------------------------------------------------------------------------
# Library
# -------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/node_id.cpp
  src/variant.cpp
  src/conversion.cpp
  src/client.cpp
  src/server.cpp
  src/subscription.cpp
  src/lifecycle_client_node.cpp
  src/type_registry.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME} PRIVATE OPCUA_ROS2_CORE_BUILDING_DLL)

# open62541pp is PRIVATE — it must not be re-exported in public headers.
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
  PRIVATE
    open62541pp::open62541pp
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    opcua_ros2_interfaces
    builtin_interfaces
    unique_identifier_msgs
    lifecycle_msgs
)

# -------------------------------------------------------------------------
# Install
# -------------------------------------------------------------------------
install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  opcua_ros2_vendor
  opcua_ros2_interfaces
  builtin_interfaces
  unique_identifier_msgs
)

# -------------------------------------------------------------------------
# Tests
# -------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_conversion test/test_conversion.cpp)
  target_link_libraries(test_conversion ${PROJECT_NAME})

  ament_add_gtest(test_client test/test_client_against_in_process_server.cpp)
  target_link_libraries(test_client ${PROJECT_NAME})
  # test_client uses open62541pp Server internally — link it privately here
  target_link_libraries(test_client open62541pp::open62541pp)

  ament_add_gtest(test_lifecycle test/test_lifecycle_node.cpp)
  target_link_libraries(test_lifecycle ${PROJECT_NAME})
  target_link_libraries(test_lifecycle open62541pp::open62541pp)
endif()

ament_package()
