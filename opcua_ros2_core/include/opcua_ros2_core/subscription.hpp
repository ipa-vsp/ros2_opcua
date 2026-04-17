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
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

#include "opcua_ros2_core/data_value.hpp"
#include "opcua_ros2_core/error.hpp"
#include "opcua_ros2_core/node_id.hpp"
#include "opcua_ros2_core/visibility_control.hpp"

#include <rclcpp/waitable.hpp>
#include <rclcpp/callback_group.hpp>

namespace opcua_ros2
{

struct OPCUA_ROS2_CORE_PUBLIC SubscriptionConfig
{
  std::chrono::milliseconds publishing_interval{100};
  uint32_t lifetime_count{10000};
  uint32_t max_keep_alive_count{10};
  uint32_t max_notifications_per_publish{0};  // 0 = unlimited
  uint8_t  priority{0};
};

struct OPCUA_ROS2_CORE_PUBLIC MonitoredItemConfig
{
  NodeId   node_id;
  uint32_t attribute_id{13};  // Value attribute
  std::chrono::milliseconds sampling_interval{50};
  uint32_t queue_size{10};
  bool     discard_oldest{true};
};

/// Handle for a single monitored item within a Subscription.
class OPCUA_ROS2_CORE_PUBLIC MonitoredItem {
public:
  using DataChangeCallback =
    std::function<void(const DataValue &, const Error * /* nullable */)>;

  void set_data_change_callback(DataChangeCallback cb);
  NodeId   node_id()       const noexcept;
  uint32_t server_handle() const noexcept;

  /// Internal use only.
  struct Impl;
  explicit MonitoredItem(std::shared_ptr<Impl> impl);

private:
  std::shared_ptr<Impl> impl_;
};

/// A subscription that integrates with the rclcpp executor via rclcpp::Waitable.
/// Notifications from the OPC UA IO thread are queued and drained on the
/// ROS executor thread bound to the provided callback group.
class OPCUA_ROS2_CORE_PUBLIC Subscription
  : public std::enable_shared_from_this<Subscription>,
  public rclcpp::Waitable
{
public:
  std::shared_ptr<MonitoredItem> add_item(MonitoredItemConfig cfg);
  void remove_item(std::shared_ptr<MonitoredItem> item);
  std::size_t item_count() const noexcept;

  // rclcpp::Waitable interface
  bool is_ready(const rcl_wait_set_t & wait_set) override;
  std::shared_ptr<void> take_data() override;
  void execute(const std::shared_ptr<void> & data) override;
  void add_to_wait_set(rcl_wait_set_t & wait_set) override;

  /// Internal use only.
  struct Impl;
  explicit Subscription(std::shared_ptr<Impl> impl);

private:
  std::shared_ptr<Impl> impl_;
};

}  // namespace opcua_ros2
