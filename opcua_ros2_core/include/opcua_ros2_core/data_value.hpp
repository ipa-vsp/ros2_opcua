// Copyright 2024 Vishnuprasad Prachandabhanu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
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
