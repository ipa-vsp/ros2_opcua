// Copyright 2024 Vishnuprasad Prachandabhanu
// SPDX-License-Identifier: Apache-2.0
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
