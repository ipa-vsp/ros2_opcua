cmake_minimum_required(VERSION 3.8)
project(open62541pp_core_libraries)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

include(ExternalProject)
ExternalProject_Add(
    open62541pp_external
    PREFIX ${CMAKE_BINARY_DIR}/_deps
    GIT_REPOSITORY https://github.com/open62541pp/open62541pp.git
    GIT_TAG v0.7.0
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
    UPDATE_DISCONNECTED TRUE
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
)

# Get the install directory of the external project
ExternalProject_Get_Property(open62541pp_external INSTALL_DIR)
set(open62541pp_DIR ${INSTALL_DIR})

if(EXISTS ${open62541pp_DIR}/src/open62541pp)
  add_subdirectory(${open62541pp_DIR}/src/open62541pp ${CMAKE_BINARY_DIR}/open62541pp)
endif()

# Export dependencies for other packages that depend on this package
ament_export_include_directories(${open62541pp_DIR}/include/)

install(
  DIRECTORY ${open62541pp_DIR}/include/
  DESTINATION include
)

install(
  DIRECTORY ${open62541pp_DIR}/lib/
  DESTINATION lib
)

install(
  DIRECTORY ${open62541pp_DIR}/share/
  DESTINATION share
)

ament_export_libraries(open62541pp)

# Test settings (if applicable)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
