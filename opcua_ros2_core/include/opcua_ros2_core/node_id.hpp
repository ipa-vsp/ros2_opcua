// Copyright 2024 Vishnuprasad Prachandabhanu
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <compare>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>
#include "opcua_ros2_core/detail/expected.hpp"
#include "opcua_ros2_core/error.hpp"
#include "opcua_ros2_core/visibility_control.hpp"

namespace opcua_ros2
{

using Guid = std::array<uint8_t, 16>;

/// OPC UA NodeId — no open62541 dependency in this header.
class OPCUA_ROS2_CORE_PUBLIC NodeId {
public:
  enum class IdentifierType : uint8_t
  {
    Numeric = 0,
    String  = 1,
    Guid    = 2,
    Opaque  = 3,
  };

  NodeId() = default;
  NodeId(uint16_t ns, uint32_t numeric) noexcept;
  NodeId(uint16_t ns, std::string s) noexcept;
  NodeId(uint16_t ns, opcua_ros2::Guid guid) noexcept;
  NodeId(uint16_t ns, std::vector<uint8_t> opaque) noexcept;

  uint16_t       namespace_index()  const noexcept;
  IdentifierType identifier_type()  const noexcept;
  uint32_t       numeric_id()       const;   ///< throws if not numeric
  const std::string & string_id()  const;             ///< throws if not string
  const opcua_ros2::Guid & guid_id()    const;        ///< throws if not guid
  const std::vector<uint8_t> & opaque_id()  const;    ///< throws if not opaque

  /// Render as "ns=X;i=Y", "ns=X;s=Y", etc.
  std::string to_string() const;

  /// Parse "ns=X;i=Y", "i=Y", "ns=X;s=Y", etc.
  static tl::expected<NodeId, Error> parse(std::string_view sv) noexcept;

  auto operator<=>(const NodeId &) const = default;

private:
  uint16_t ns_{0};
  std::variant<uint32_t, std::string, opcua_ros2::Guid, std::vector<uint8_t>> id_{uint32_t{0}};
};

}  // namespace opcua_ros2
