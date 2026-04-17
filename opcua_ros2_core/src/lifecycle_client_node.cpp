// Copyright 2024 Vishnuprasad Prachandabhanu
// SPDX-License-Identifier: Apache-2.0
#include "opcua_ros2_core/lifecycle_client_node.hpp"

#include <stdexcept>

namespace opcua_ros2
{

LifecycleClientNode::LifecycleClientNode(
  const std::string & name,
  const rclcpp::NodeOptions & opts)
: rclcpp_lifecycle::LifecycleNode(name, opts)
{
  declare_opcua_parameters();
}

void LifecycleClientNode::declare_opcua_parameters()
{
  declare_parameter("endpoint", rclcpp::PARAMETER_STRING);
  declare_parameter("security_mode", static_cast<int>(SecurityMode::None));
  declare_parameter("security_policy", std::string{});
  declare_parameter("username", std::string{});
  declare_parameter("password", std::string{});
  declare_parameter("reconnect_backoff_ms", 5000);
}

Client & LifecycleClientNode::client()
{
  if (!client_) {throw std::runtime_error("Client not yet configured");}
  return *client_;
}

LifecycleClientNode::CallbackReturn
LifecycleClientNode::on_configure(const rclcpp_lifecycle::State &)
{
  ClientConfig cfg;
  cfg.endpoint_url = get_parameter("endpoint").as_string();
  cfg.security_mode = static_cast<SecurityMode>(get_parameter("security_mode").as_int());
  cfg.security_policy_uri = get_parameter("security_policy").as_string();
  cfg.username = get_parameter("username").as_string();
  cfg.password = get_parameter("password").as_string();
  cfg.reconnect_backoff = std::chrono::milliseconds{
    get_parameter("reconnect_backoff_ms").as_int()};

  client_ = std::make_unique<Client>(cfg);
  return on_opcua_configured(*client_);
}

LifecycleClientNode::CallbackReturn
LifecycleClientNode::on_activate(const rclcpp_lifecycle::State &)
{
  auto res = client_->connect();
  if (!res) {
    RCLCPP_ERROR(get_logger(), "OPC UA connect failed: %s", res.error().message.c_str());
    return CallbackReturn::FAILURE;
  }
  return on_opcua_activated(*client_);
}

LifecycleClientNode::CallbackReturn
LifecycleClientNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  auto ret = on_opcua_deactivated(*client_);
  client_->disconnect();
  return ret;
}

LifecycleClientNode::CallbackReturn
LifecycleClientNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  client_.reset();
  return CallbackReturn::SUCCESS;
}

LifecycleClientNode::CallbackReturn
LifecycleClientNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (client_) {client_->disconnect(); client_.reset();}
  return CallbackReturn::SUCCESS;
}

}  // namespace opcua_ros2
