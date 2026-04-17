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
