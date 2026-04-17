# open62541pp_datatype_headers-extras.cmake
#
# Convenience wrapper that exposes upstream open62541's NodeSet-to-header
# generator (dtgen) as a CMake function for downstream users. It is meant for
# small projects that want a handful of generated data-type headers without
# pulling in the full opcua_ros2_idl pipeline.
#
# Usage:
#   find_package(opcua_ros2_vendor REQUIRED)
#
#   opcua_ros2_generate_datatype_headers(my_types
#     NODESET           ${CMAKE_CURRENT_SOURCE_DIR}/config/my_types.xml
#     NAMESPACE_URI     "http://example.org/my_types/"
#     OUTPUT_DIR        ${CMAKE_CURRENT_BINARY_DIR}/generated
#   )
#
# The resulting target `my_types` is an INTERFACE library that exposes the
# generated include directory; link it into your consumers.

if(COMMAND opcua_ros2_generate_datatype_headers)
  return()
endif()

# Locate the dtgen script shipped by upstream open62541. ament_vendor installs
# the sources into opt/<pkg>/share/... so we search both the install tree and
# the vendor build prefix.
find_program(OPCUA_ROS2_DTGEN
  NAMES nodeset_compiler.py nodeset_compiler
  HINTS
    ${opcua_ros2_vendor_DIR}/../../../opt/opcua_ros2_vendor/share/open62541/tools/nodeset_compiler
    ${CMAKE_INSTALL_PREFIX}/opt/opcua_ros2_vendor/share/open62541/tools/nodeset_compiler
  DOC "Upstream open62541 nodeset_compiler entrypoint")

function(opcua_ros2_generate_datatype_headers target)
  set(_options)
  set(_one_value NODESET NAMESPACE_URI OUTPUT_DIR)
  set(_multi_value DEPENDS)
  cmake_parse_arguments(ARG "${_options}" "${_one_value}" "${_multi_value}" ${ARGN})

  if(NOT ARG_NODESET)
    message(FATAL_ERROR "opcua_ros2_generate_datatype_headers(${target}): NODESET is required")
  endif()
  if(NOT ARG_OUTPUT_DIR)
    set(ARG_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/${target}_generated)
  endif()

  file(MAKE_DIRECTORY ${ARG_OUTPUT_DIR})

  if(NOT OPCUA_ROS2_DTGEN)
    message(WARNING
      "opcua_ros2_generate_datatype_headers(${target}): nodeset_compiler was not "
      "found in the vendored open62541 install. The target will be declared but "
      "no generation command will be attached. Install with BUILD_TOOLS=ON or "
      "switch to opcua_ros2_idl for full codegen.")
  else()
    add_custom_command(
      OUTPUT  ${ARG_OUTPUT_DIR}/${target}_generated.stamp
      COMMAND ${Python3_EXECUTABLE} ${OPCUA_ROS2_DTGEN}
              --types-xml ${ARG_NODESET}
              --output-dir ${ARG_OUTPUT_DIR}
      COMMAND ${CMAKE_COMMAND} -E touch ${ARG_OUTPUT_DIR}/${target}_generated.stamp
      DEPENDS ${ARG_NODESET} ${ARG_DEPENDS}
      COMMENT "Generating OPC UA datatype headers for ${target}"
      VERBATIM)
    add_custom_target(${target}_generate DEPENDS ${ARG_OUTPUT_DIR}/${target}_generated.stamp)
  endif()

  add_library(${target} INTERFACE)
  target_include_directories(${target} INTERFACE ${ARG_OUTPUT_DIR})
  if(TARGET ${target}_generate)
    add_dependencies(${target} ${target}_generate)
  endif()
endfunction()
