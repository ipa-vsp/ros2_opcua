cmake_minimum_required(VERSION 3.16)
project(opcua_ros2_fake_server VERSION 0.1.0 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
