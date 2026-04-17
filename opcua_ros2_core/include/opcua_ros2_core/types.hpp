// Copyright 2024 Vishnuprasad Prachandabhanu
// SPDX-License-Identifier: Apache-2.0
// Convenience umbrella for all lightweight OPC UA value types.
#pragma once

#include "opcua_ros2_core/node_id.hpp"
#include "opcua_ros2_core/status_code.hpp"
#include "opcua_ros2_core/error.hpp"

#include <cstdint>
#include <string>
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
