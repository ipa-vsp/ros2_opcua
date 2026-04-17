cmake_minimum_required(VERSION 3.16)
project(opcua_ros2_vendor VERSION 0.1.0 LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)
find_package(OpenSSL REQUIRED)

ament_vendor(opcua_ros2_vendor
  VCS_URL      https://github.com/open62541pp/open62541pp.git
  VCS_VERSION  4bc85304897d3fd23bb76954b08380d06b555bc3
  CMAKE_ARGS
    -DUAPP_BUILD_EXAMPLES:BOOL=OFF
    -DUAPP_BUILD_TESTS:BOOL=OFF
    -DUAPP_ENABLE_ENCRYPTION:BOOL=ON
    -DUA_ENABLE_ENCRYPTION:STRING=OPENSSL
    -DUA_ENABLE_JSON_ENCODING:BOOL=ON
    -DUA_ENABLE_SUBSCRIPTIONS:BOOL=ON
    -DUA_ENABLE_SUBSCRIPTIONS_EVENTS:BOOL=ON
    -DUA_ENABLE_METHODCALLS:BOOL=ON
    -DUA_ENABLE_DISCOVERY:BOOL=ON
    -DBUILD_SHARED_LIBS:BOOL=ON
    -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON
    -DCMAKE_BUILD_TYPE:STRING=$<IF:$<CONFIG:Debug>,Debug,Release>
)

ament_export_dependencies(OpenSSL)

# Install the datatype-header helper scripts & templates for downstream users
# who want to generate code straight from NodeSet XML without the full
# opcua_ros2_idl pipeline.
install(DIRECTORY cmake/
        DESTINATION share/${PROJECT_NAME}/cmake)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(smoke_test test/smoke_test.cpp)
  if(TARGET smoke_test)
    # The vendored imported target only becomes available after the
    # ament_vendor ExternalProject finishes installing. Guard the linkage
    # so a clean first-pass configure does not error out.
    if(TARGET open62541pp::open62541pp)
      target_link_libraries(smoke_test PRIVATE open62541pp::open62541pp)
    else()
      # Add ExternalProject dependency so the test only builds after vendor install.
      add_dependencies(smoke_test ${PROJECT_NAME})
      target_include_directories(smoke_test PRIVATE
        ${CMAKE_INSTALL_PREFIX}/include)
      target_link_directories(smoke_test PRIVATE
        ${CMAKE_INSTALL_PREFIX}/lib)
      target_link_libraries(smoke_test open62541pp open62541)
    endif()
  endif()
endif()

ament_package(CONFIG_EXTRAS
  "cmake/open62541pp_datatype_headers-extras.cmake")
