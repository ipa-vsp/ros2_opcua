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

#include <cstdint>
#include <memory>
#include <string>

#include "opcua_ros2_core/data_value.hpp"
#include "opcua_ros2_core/detail/expected.hpp"
#include "opcua_ros2_core/error.hpp"
#include "opcua_ros2_core/node_id.hpp"
#include "opcua_ros2_core/visibility_control.hpp"

namespace opcua_ros2
{

/// Minimal configuration for spinning up a local OPC UA server.
struct OPCUA_ROS2_CORE_PUBLIC ServerConfig
{
  uint16_t    port{4840};
  std::string application_name{"opcua_ros2_server"};
  std::string application_uri{"urn:opcua_ros2:server"};
};

/// Wrapper around opcua::Server.
/// Starts/stops the server event loop on its own thread.
class OPCUA_ROS2_CORE_PUBLIC Server {
public:
  explicit Server(ServerConfig cfg = {});
  ~Server();

  Server(const Server &) = delete;
  Server & operator=(const Server &) = delete;

  tl::expected<void, Error> start();
  tl::expected<void, Error> stop();
  bool is_running() const noexcept;

  /// Write a value directly into the server's address space.
  tl::expected<void, Error> write_variable(const NodeId & id, const DataValue & dv);

  /// Read a value from the server's address space.
  tl::expected<DataValue, Error> read_variable(const NodeId & id);

  uint16_t port() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace opcua_ros2
