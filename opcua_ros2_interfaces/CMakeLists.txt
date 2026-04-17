cmake_minimum_required(VERSION 3.16)
project(opcua_ros2_interfaces VERSION 0.1.0 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(action_msgs REQUIRED)

set(msg_files
  "msg/NodeId.msg"
  "msg/QualifiedName.msg"
  "msg/LocalizedText.msg"
  "msg/StatusCode.msg"
  "msg/Variant.msg"
  "msg/DataValue.msg"
  "msg/ReferenceDescription.msg"
  "msg/BrowseResult.msg"
  "msg/ApplicationDescription.msg"
  "msg/EndpointDescription.msg"
)

set(srv_files
  "srv/Read.srv"
  "srv/Write.srv"
  "srv/Browse.srv"
  "srv/TranslateBrowsePath.srv"
  "srv/Call.srv"
  "srv/HistoryRead.srv"
)

set(action_files
  "action/Connect.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces unique_identifier_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # managed by file headers
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_variant_roundtrip test/test_variant_roundtrip.py)
  ament_add_pytest_test(test_nodeid_equality   test/test_nodeid_equality.py)
endif()

ament_package()
