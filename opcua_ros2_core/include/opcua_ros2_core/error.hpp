// Copyright 2024 Vishnuprasad Prachandabhanu
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <source_location>
#include <string>
#include "opcua_ros2_core/status_code.hpp"
#include "opcua_ros2_core/visibility_control.hpp"

namespace opcua_ros2
{

/// Error returned via tl::expected; carries a StatusCode plus a descriptive message.
struct OPCUA_ROS2_CORE_PUBLIC Error
{
  StatusCode status;
  std::string message;
  std::source_location loc = std::source_location::current();

  static Error from_status(
    uint32_t raw, std::string msg = {},
    std::source_location l = std::source_location::current())
  {
    StatusCode sc;
    sc.value = raw;
    return Error{sc, std::move(msg), l};
  }
};

}  // namespace opcua_ros2
