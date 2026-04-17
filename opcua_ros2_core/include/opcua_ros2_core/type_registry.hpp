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
