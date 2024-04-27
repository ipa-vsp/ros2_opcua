cmake_minimum_required(VERSION 3.8)
project(open62541pp_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)
# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)

include(FetchContent)

FetchContent_Declare(
  open62541pp
  GIT_REPOSITORY https://github.com/open62541pp/open62541pp.git
  GIT_TAG e8fb97e704447c52538a8b40d1d3295c12d9dd81
  GIT_PROGRESS TRUE
  GIT_SHALLOW TRUE)

# # # Set the CMake arguments for the library
set(UAPP_BUILD_EXAMPLES ON CACHE BOOL "Build examples for open62541pp")
set(UAPP_BUILD_TESTS ON CACHE BOOL "Build tests for open62541pp")
set(CMAKE_BUILD_TYPE Debug CACHE STRING "Set the build type")
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries")
set(UA_ENABLE_ENCRYPTION OPENSSL)
# set(UA_ENABLE_ENCRYPTION STREQUAL "OPENSSL")

FetchContent_GetProperties(open62541pp)
if(NOT open62541pp_POPULATED)
  FetchContent_Populate(open62541pp)
  add_subdirectory(${open62541pp_SOURCE_DIR} ${open62541pp_BINARY_DIR})
endif()

ament_vendor(
  open62541pp_vendor
  SATISFIED ${open62541pp_POPULATED}
  VCS_URL https://github.com/open62541pp/open62541pp.git
  VCS_VERSION e8fb97e704447c52538a8b40d1d3295c12d9dd81
  CMAKE_ARGS
    -DUAPP_BUILD_EXAMPLES:BOOL=OFF
    -DUAPP_BUILD_TESTS:BOOL=OFF
    -DCMAKE_BUILD_TYPE=Debug
    -DBUILD_SHARED_LIBS:BOOL=ON
)

install(
  TARGETS open62541pp
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Datatype headers generator
set(datatype_cmake_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include("${datatype_cmake_DIR}/open62541pp_datatype_headers-extras.cmake" NO_POLICY_SCOPE)
install(
    DIRECTORY cmake
    DESTINATION share/${PROJECT_NAME}
)
  
ament_python_install_package(dtgen SCRIPTS_DESTINATION lib/dtgen)
  
# install entry-point script(s) in bin as well
install(
      DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_python/dtgen/scripts/
      DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
      USE_SOURCE_PERMISSIONS
)
  
# install(DIRECTORY templates
#   DESTINATION share/${PROJECT_NAME}/templates
# )

# # Export the necessary directories
ament_export_include_directories(include)
ament_export_libraries(open62541pp)
ament_export_targets(export_${PROJECT_NAME})

# if(BUILD_TESTING) find_package(ament_lint_auto REQUIRED) list(APPEND
# AMENT_LINT_AUTO_EXCLUDE ament_cmake_cppcheck )
# ament_lint_auto_find_test_dependencies() endif()

ament_package(
  CONFIG_EXTRAS
  "${datatype_cmake_DIR}/open62541pp_datatype_headers-extras.cmake"
)
