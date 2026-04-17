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
// Free-function conversions between our types and opcua_ros2_interfaces messages.
// No open62541 dependency in this header.
#pragma once

#include "opcua_ros2_core/data_value.hpp"
#include "opcua_ros2_core/node_id.hpp"
#include "opcua_ros2_core/status_code.hpp"
#include "opcua_ros2_core/types.hpp"
#include "opcua_ros2_core/variant.hpp"
#include "opcua_ros2_core/visibility_control.hpp"

#include "opcua_ros2_interfaces/msg/data_value.hpp"
#include "opcua_ros2_interfaces/msg/node_id.hpp"
#include "opcua_ros2_interfaces/msg/qualified_name.hpp"
#include "opcua_ros2_interfaces/msg/localized_text.hpp"
#include "opcua_ros2_interfaces/msg/status_code.hpp"
#include "opcua_ros2_interfaces/msg/variant.hpp"

namespace opcua_ros2::conversion
{

OPCUA_ROS2_CORE_PUBLIC
opcua_ros2_interfaces::msg::NodeId to_ros(const NodeId & id);

OPCUA_ROS2_CORE_PUBLIC
NodeId to_core(const opcua_ros2_interfaces::msg::NodeId & msg);

OPCUA_ROS2_CORE_PUBLIC
opcua_ros2_interfaces::msg::StatusCode to_ros(const StatusCode & sc);

OPCUA_ROS2_CORE_PUBLIC
StatusCode to_core(const opcua_ros2_interfaces::msg::StatusCode & msg);

OPCUA_ROS2_CORE_PUBLIC
opcua_ros2_interfaces::msg::QualifiedName to_ros(const QualifiedName & qn);

OPCUA_ROS2_CORE_PUBLIC
QualifiedName to_core(const opcua_ros2_interfaces::msg::QualifiedName & msg);

OPCUA_ROS2_CORE_PUBLIC
opcua_ros2_interfaces::msg::LocalizedText to_ros(const LocalizedText & lt);

OPCUA_ROS2_CORE_PUBLIC
LocalizedText to_core(const opcua_ros2_interfaces::msg::LocalizedText & msg);

OPCUA_ROS2_CORE_PUBLIC
opcua_ros2_interfaces::msg::Variant to_ros(const Variant & v);

OPCUA_ROS2_CORE_PUBLIC
Variant to_core(const opcua_ros2_interfaces::msg::Variant & msg);

OPCUA_ROS2_CORE_PUBLIC
opcua_ros2_interfaces::msg::DataValue to_ros(const DataValue & dv);

OPCUA_ROS2_CORE_PUBLIC
DataValue to_core(const opcua_ros2_interfaces::msg::DataValue & msg);

}  // namespace opcua_ros2::conversion
