cmake_minimum_required(VERSION 3.8)
project(opcua_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-dev)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(open62541pp REQUIRED)
find_package(open62541pp_vendor REQUIRED)

generate_datatype_headers(first)

set(THIS_PACKAGE_INCLUDE_DEPENDS rclcpp open62541pp open62541pp_vendor)

# Tutorial examples
add_executable(example_server_node src/example_server_node.cpp)
target_include_directories(
  example_server_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(example_server_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(example_client_node src/example_client_node.cpp)
target_include_directories(
  example_client_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(example_client_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS example_server_node example_client_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

# if(BUILD_TESTING) find_package(ament_lint_auto REQUIRED) # the following line
# skips the linter which checks for copyrights set(ament_cmake_copyright_FOUND
# TRUE) set(ament_cmake_cpplint_FOUND TRUE)
# ament_lint_auto_find_test_dependencies() endif()

ament_package()
