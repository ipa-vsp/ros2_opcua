// Copyright 2024 Vishnuprasad Prachandabhanu
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <optional>
#include "opcua_ros2_core/status_code.hpp"
#include "opcua_ros2_core/variant.hpp"
#include "opcua_ros2_core/visibility_control.hpp"


namespace opcua_ros2
{

/// OPC UA DataValue — value + status + timestamps.
/// Mirrors opcua_ros2_interfaces/msg/DataValue without depending on open62541.
struct OPCUA_ROS2_CORE_PUBLIC DataValue
{
  Variant    value;
  StatusCode status;

  // Timestamps stored as (seconds, nanoseconds) pairs; zero = not present.
  int32_t  source_timestamp_sec{0};
  uint32_t source_timestamp_nanosec{0};
  uint16_t source_picoseconds{0};

  int32_t  server_timestamp_sec{0};
  uint32_t server_timestamp_nanosec{0};
  uint16_t server_picoseconds{0};

  bool has_value()            const noexcept {return !value.is_null();}
  bool has_source_timestamp() const noexcept
  {
    return source_timestamp_sec != 0 || source_timestamp_nanosec != 0;
  }
  bool has_server_timestamp() const noexcept
  {
    return server_timestamp_sec != 0 || server_timestamp_nanosec != 0;
  }
};

}  // namespace opcua_ros2
