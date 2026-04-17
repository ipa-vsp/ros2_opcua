# opcua_ros2_vendor

Vendored build of [open62541pp](https://github.com/open62541pp/open62541pp)
(a modern C++ wrapper around [open62541](https://open62541.org/)) for the
ROS 2 ecosystem.

## What it provides

- An `ament_vendor` build of `open62541pp` at the pinned SHA
  `6b83f185e0fbc78f98647ff4afd848b8a17cbc60`, with OpenSSL-backed encryption
  and shared libraries.
- The exported CMake imported target `open62541pp::open62541pp`. Downstream
  packages call `find_package(opcua_ros2_vendor REQUIRED)` and link against
  that target — they should not talk to `open62541` directly.
- The `open62541pp_datatype_headers-extras.cmake` helper for users who want
  to run the upstream NodeSet-to-headers compiler without pulling in the full
  `opcua_ros2_idl` pipeline.

## Build flags passed to the vendored project

| Flag                                  | Value    | Reason                                              |
|---------------------------------------|----------|-----------------------------------------------------|
| `UAPP_BUILD_EXAMPLES`                 | `OFF`    | Vendored build ships no upstream demos.             |
| `UAPP_BUILD_TESTS`                    | `OFF`    | Upstream tests are out of scope.                    |
| `UAPP_ENABLE_ENCRYPTION`              | `ON`     | Required by the ros2_control_opcua demos.           |
| `UA_ENABLE_ENCRYPTION`                | `OPENSSL`| OpenSSL backend for consistency with ROS 2 Jazzy.   |
| `UA_ENABLE_JSON_ENCODING`             | `ON`     | Required for the JSON fault-injection path.         |
| `UA_ENABLE_SUBSCRIPTIONS[_EVENTS]`    | `ON`     | Required by `opcua_ros2_core`.                      |
| `UA_ENABLE_METHODCALLS`               | `ON`     | Required by `opcua_ros2_examples/method_caller`.    |
| `UA_ENABLE_DISCOVERY`                 | `ON`     | Required by `Connect.action` progress reporting.    |
| `BUILD_SHARED_LIBS`                   | `ON`     | ROS 2 convention; keeps ABI swappable.              |

## Bumping the pin

1. Update `VCS_VERSION` in `CMakeLists.txt`.
2. Bump this package's patch version in `package.xml`.
3. Document the change in `CHANGELOG.rst`.
4. Run `colcon build --packages-up-to opcua_ros2_vendor` and
   `colcon test --packages-select opcua_ros2_vendor`.
