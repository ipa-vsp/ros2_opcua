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

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <future>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "opcua_ros2_core/data_value.hpp"
#include "opcua_ros2_core/detail/expected.hpp"
#include "opcua_ros2_core/error.hpp"
#include "opcua_ros2_core/node_id.hpp"
#include "opcua_ros2_core/subscription.hpp"
#include "opcua_ros2_core/visibility_control.hpp"

#include <rclcpp/callback_group.hpp>

namespace opcua_ros2
{

/// Security modes — mirrors OPC UA spec; no open62541 leakage.
enum class SecurityMode : uint8_t { None = 1, Sign = 2, SignAndEncrypt = 3 };

struct OPCUA_ROS2_CORE_PUBLIC BrowseResultEntry
{
  NodeId         node_id;
  std::string    browse_name;
  std::string    display_name;
  uint32_t       node_class{0};
  NodeId         reference_type_id;
  bool           is_forward{true};
};

using BrowseResult = std::vector<BrowseResultEntry>;

struct OPCUA_ROS2_CORE_PUBLIC BrowseRequest
{
  NodeId   node_id;
  uint32_t browse_direction{0};     // 0=Forward, 1=Inverse, 2=Both
  NodeId   reference_type_id;       // empty = all references
  bool     include_subtypes{true};
  uint32_t node_class_mask{0};      // 0 = all node classes
  uint32_t result_mask{63};         // 63 = all fields
};

using CallResult = std::vector<Variant>;

/// Configuration for the OPC UA client.
struct OPCUA_ROS2_CORE_PUBLIC ClientConfig
{
  std::string endpoint_url;
  std::string application_uri{"urn:opcua_ros2:client"};
  std::chrono::milliseconds requested_publishing_interval{100};
  std::chrono::milliseconds session_timeout{60'000};
  std::chrono::milliseconds reconnect_backoff{5'000};
  SecurityMode security_mode{SecurityMode::None};
  std::string  security_policy_uri;
  std::string  username;
  std::string  password;
  std::filesystem::path certificate_file;
  std::filesystem::path private_key_file;
};

/// Thread-safe OPC UA client.
/// All OPC UA I/O runs on a dedicated thread started in connect().
/// ROS callbacks are dispatched to the supplied rclcpp::CallbackGroup.
class OPCUA_ROS2_CORE_PUBLIC Client {
public:
  explicit Client(ClientConfig cfg);
  ~Client();

  Client(const Client &) = delete;
  Client & operator=(const Client &) = delete;

  // --- lifecycle ---
  tl::expected<void, Error> connect();
  tl::expected<void, Error> disconnect();
  bool                       is_connected() const noexcept;

  // --- synchronous service calls (avoid on ROS executor threads) ---
  tl::expected<DataValue, Error> read(const NodeId & id, uint32_t attribute_id = 13);
  tl::expected<void, Error> write(
    const NodeId & id, const DataValue & dv,
    uint32_t attribute_id = 13);
  tl::expected<BrowseResult, Error> browse(const BrowseRequest & req);
  tl::expected<CallResult, Error> call(
    const NodeId & object_id,
    const NodeId & method_id,
    std::span<const Variant> inputs);

  // --- asynchronous service calls (safe from any thread) ---
  std::future<tl::expected<DataValue, Error>> async_read(
    const NodeId & id,
    uint32_t attribute_id = 13);
  std::future<tl::expected<void, Error>> async_write(
    const NodeId & id,
    const DataValue & dv,
    uint32_t attribute_id = 13);

  // --- subscription factory ---
  std::shared_ptr<Subscription> create_subscription(
    SubscriptionConfig cfg,
    rclcpp::CallbackGroup::SharedPtr cb_group);

  const ClientConfig & config() const noexcept;
  uint64_t             session_id() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace opcua_ros2
