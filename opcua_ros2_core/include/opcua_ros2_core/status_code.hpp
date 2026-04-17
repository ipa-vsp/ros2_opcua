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
#include <string>
#include "opcua_ros2_core/visibility_control.hpp"

namespace opcua_ros2
{

/// Lightweight OPC UA status code (no open62541 dependency).
struct OPCUA_ROS2_CORE_PUBLIC StatusCode
{
  static constexpr uint32_t Good = 0x00000000U;
  static constexpr uint32_t BadUnexpected = 0x80010000U;
  static constexpr uint32_t BadDisconnect = 0x80020000U;
  static constexpr uint32_t BadNodeIdUnknown = 0x80340000U;

  uint32_t value{Good};
  std::string name;  ///< human-readable (populated by conversion helpers)

  bool is_good()    const noexcept {return (value & 0xC0000000U) == 0x00000000U;}
  bool is_uncertain() const noexcept {return (value & 0xC0000000U) == 0x40000000U;}
  bool is_bad()     const noexcept {return (value & 0xC0000000U) != 0x00000000U && !is_uncertain();}

  bool operator==(const StatusCode &) const = default;
};

}  // namespace opcua_ros2
