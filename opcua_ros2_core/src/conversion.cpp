// Copyright 2024 Vishnuprasad Prachandabhanu
// SPDX-License-Identifier: Apache-2.0
#include "opcua_ros2_core/conversion.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "opcua_ros2_interfaces/msg/localized_text.hpp"
#include "opcua_ros2_interfaces/msg/node_id.hpp"
#include "opcua_ros2_interfaces/msg/qualified_name.hpp"
#include "opcua_ros2_interfaces/msg/status_code.hpp"

namespace opcua_ros2::conversion
{

opcua_ros2_interfaces::msg::NodeId to_ros(const NodeId & id)
{
  opcua_ros2_interfaces::msg::NodeId msg;
  msg.namespace_index = id.namespace_index();
  msg.identifier_type = static_cast<uint8_t>(id.identifier_type());
  switch (id.identifier_type()) {
    case NodeId::IdentifierType::Numeric:
      msg.numeric_id = id.numeric_id(); break;
    case NodeId::IdentifierType::String:
      msg.string_id = id.string_id();  break;
    case NodeId::IdentifierType::Guid: {
        const auto & g = id.guid_id();
        std::copy(g.begin(), g.end(), msg.guid_id.uuid.begin());
        break;
      }
    case NodeId::IdentifierType::Opaque: {
        const auto & op = id.opaque_id();
        msg.opaque_id.assign(op.begin(), op.end());
        break;
      }
  }
  return msg;
}

NodeId to_core(const opcua_ros2_interfaces::msg::NodeId & msg)
{
  switch (msg.identifier_type) {
    case 0: return NodeId{msg.namespace_index, msg.numeric_id};
    case 1: return NodeId{msg.namespace_index, msg.string_id};
    case 2: {
        Guid g;
        std::copy(msg.guid_id.uuid.begin(), msg.guid_id.uuid.end(), g.begin());
        return NodeId{msg.namespace_index, g};
      }
    default: {
        return NodeId{msg.namespace_index,
          std::vector<uint8_t>(msg.opaque_id.begin(), msg.opaque_id.end())};
      }
  }
}

opcua_ros2_interfaces::msg::StatusCode to_ros(const StatusCode & sc)
{
  opcua_ros2_interfaces::msg::StatusCode msg;
  msg.value = sc.value;
  msg.name = sc.name;
  return msg;
}

StatusCode to_core(const opcua_ros2_interfaces::msg::StatusCode & msg)
{
  StatusCode sc;
  sc.value = msg.value;
  sc.name = msg.name;
  return sc;
}

opcua_ros2_interfaces::msg::QualifiedName to_ros(const QualifiedName & qn)
{
  opcua_ros2_interfaces::msg::QualifiedName msg;
  msg.namespace_index = qn.namespace_index;
  msg.name = qn.name;
  return msg;
}

QualifiedName to_core(const opcua_ros2_interfaces::msg::QualifiedName & msg)
{
  return QualifiedName{msg.namespace_index, msg.name};
}

opcua_ros2_interfaces::msg::LocalizedText to_ros(const LocalizedText & lt)
{
  opcua_ros2_interfaces::msg::LocalizedText msg;
  msg.locale = lt.locale;
  msg.text = lt.text;
  return msg;
}

LocalizedText to_core(const opcua_ros2_interfaces::msg::LocalizedText & msg)
{
  return LocalizedText{msg.locale, msg.text};
}

opcua_ros2_interfaces::msg::Variant to_ros(const Variant & v)
{
  return v.to_ros();
}

Variant to_core(const opcua_ros2_interfaces::msg::Variant & msg)
{
  return Variant::from_ros(msg);
}

opcua_ros2_interfaces::msg::DataValue to_ros(const DataValue & dv)
{
  opcua_ros2_interfaces::msg::DataValue msg;
  msg.value = dv.value.to_ros();
  msg.status = to_ros(dv.status);
  msg.source_timestamp.sec = dv.source_timestamp_sec;
  msg.source_timestamp.nanosec = dv.source_timestamp_nanosec;
  msg.source_picoseconds = dv.source_picoseconds;
  msg.server_timestamp.sec = dv.server_timestamp_sec;
  msg.server_timestamp.nanosec = dv.server_timestamp_nanosec;
  msg.server_picoseconds = dv.server_picoseconds;
  return msg;
}

DataValue to_core(const opcua_ros2_interfaces::msg::DataValue & msg)
{
  DataValue dv;
  dv.value = Variant::from_ros(msg.value);
  dv.status = to_core(msg.status);
  dv.source_timestamp_sec = msg.source_timestamp.sec;
  dv.source_timestamp_nanosec = msg.source_timestamp.nanosec;
  dv.source_picoseconds = msg.source_picoseconds;
  dv.server_timestamp_sec = msg.server_timestamp.sec;
  dv.server_timestamp_nanosec = msg.server_timestamp.nanosec;
  dv.server_picoseconds = msg.server_picoseconds;
  return dv;
}

}  // namespace opcua_ros2::conversion
