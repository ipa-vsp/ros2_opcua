cmake_minimum_required(VERSION 3.8)
project(opcua_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(open62541pp REQUIRED)
find_package(open62541pp_vendor REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  open62541pp
  open62541pp_vendor
)

# Tutorial examples
add_executable(example_server_node src/example_server_node.cpp)
target_include_directories(example_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(example_server_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


add_executable(example_client_node src/example_client_node.cpp)
target_include_directories(example_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(example_client_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS example_server_node example_client_node
  DESTINATION lib/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   # set(ament_cmake_cpplint_FOUND FALSE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
