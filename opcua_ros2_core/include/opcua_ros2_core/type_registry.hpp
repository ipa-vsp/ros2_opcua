// Copyright 2024 Vishnuprasad Prachandabhanu
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "opcua_ros2_core/node_id.hpp"
#include "opcua_ros2_core/visibility_control.hpp"

namespace opcua_ros2
{

using Encoder = std::function<std::vector<std::byte>(const void * typed_msg)>;
using Decoder = std::function<void(std::span<const std::byte> body, void * typed_msg)>;

/// Singleton registry for structured DataType codecs.
/// opcua_ros2_idl-generated translation units register codecs at static-init.
class OPCUA_ROS2_CORE_PUBLIC TypeRegistry {
public:
  static TypeRegistry & instance();

  void register_codec(
    const NodeId & data_type_id,
    std::string_view ros_type_name,
    Encoder encoder,
    Decoder decoder);

  std::optional<Encoder> find_encoder(const NodeId & id) const;
  std::optional<Decoder> find_decoder(const NodeId & id) const;

  /// Clear all registered codecs (test helper).
  void clear();

private:
  TypeRegistry();
  ~TypeRegistry();

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace opcua_ros2
