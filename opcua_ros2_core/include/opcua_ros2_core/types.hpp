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
// Convenience umbrella for all lightweight OPC UA value types.
#pragma once

#include <cstdint>
#include <string>

#include "opcua_ros2_core/error.hpp"
#include "opcua_ros2_core/node_id.hpp"
#include "opcua_ros2_core/status_code.hpp"
#include "opcua_ros2_core/visibility_control.hpp"

namespace opcua_ros2
{

/// OPC UA QualifiedName: namespace index + name.
struct OPCUA_ROS2_CORE_PUBLIC QualifiedName
{
  uint16_t    namespace_index{0};
  std::string name;
  bool operator==(const QualifiedName &) const = default;
};

/// OPC UA LocalizedText: locale tag + text.
struct OPCUA_ROS2_CORE_PUBLIC LocalizedText
{
  std::string locale;
  std::string text;
  bool operator==(const LocalizedText &) const = default;
};

}  // namespace opcua_ros2
