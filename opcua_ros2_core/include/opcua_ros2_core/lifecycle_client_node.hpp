// Copyright 2024 Vishnuprasad Prachandabhanu
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <string>

#include "opcua_ros2_core/client.hpp"
#include "opcua_ros2_core/visibility_control.hpp"

#include <rclcpp/node_options.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace opcua_ros2
{

/// Base lifecycle node that manages an OPC UA Client through the ROS 2
/// lifecycle state machine.  Subclasses override the on_opcua_* hooks.
class OPCUA_ROS2_CORE_PUBLIC LifecycleClientNode
  : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit LifecycleClientNode(
    const std::string & name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  // Subclass override points.
  virtual CallbackReturn on_opcua_configured(Client &) {return CallbackReturn::SUCCESS;}
  virtual CallbackReturn on_opcua_activated(Client &) {return CallbackReturn::SUCCESS;}
  virtual CallbackReturn on_opcua_deactivated(Client &) {return CallbackReturn::SUCCESS;}

  Client & client();

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) final;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &)  final;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)final;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)   final;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)  final;

private:
  /// Declared ROS parameters:
  ///   endpoint          (string)
  ///   security_mode     (int, default 1 = None)
  ///   security_policy   (string)
  ///   username          (string)
  ///   password          (string)
  ///   reconnect_backoff_ms (int, default 5000)
  void declare_opcua_parameters();

  std::unique_ptr<Client> client_;
};

}  // namespace opcua_ros2
