// Copyright 2024 Vishnuprasad Prachandabhanu
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "opcua_ros2_core/lifecycle_client_node.hpp"
#include "opcua_ros2_core/server.hpp"

using namespace opcua_ros2;
using namespace std::chrono_literals;

struct TestNode : LifecycleClientNode
{
  int opcua_configure_calls = 0;
  int opcua_activate_calls = 0;
  int opcua_deactivate_calls = 0;

  explicit TestNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions{})
  : LifecycleClientNode("test_lifecycle_client", opts) {}

  CallbackReturn on_opcua_configured(Client &) override
  {
    ++opcua_configure_calls;
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_opcua_activated(Client &) override
  {
    ++opcua_activate_calls;
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_opcua_deactivated(Client &) override
  {
    ++opcua_deactivate_calls;
    return CallbackReturn::SUCCESS;
  }
};

class LifecycleTest : public ::testing::Test {
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    ServerConfig sc;
    sc.port = 14841;
    server_ = std::make_unique<Server>(sc);
    ASSERT_TRUE(server_->start().has_value());
    std::this_thread::sleep_for(100ms);
  }

  void TearDown() override
  {
    server_->stop();
    rclcpp::shutdown();
  }

  std::unique_ptr<Server> server_;
};

TEST_F(LifecycleTest, FullLifecycle) {
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("endpoint", "opc.tcp://localhost:14841");

  auto node = std::make_shared<TestNode>(opts);

  // configure
  auto res = node->trigger_transition(
    rclcpp_lifecycle::Transition{
    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE});
  EXPECT_EQ(res.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    << "expected INACTIVE after configure";
  EXPECT_EQ(node->opcua_configure_calls, 1);

  // activate
  res = node->trigger_transition(
    rclcpp_lifecycle::Transition{
    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE});
  EXPECT_EQ(res.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(node->opcua_activate_calls, 1);

  // deactivate
  res = node->trigger_transition(
    rclcpp_lifecycle::Transition{
    lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE});
  EXPECT_EQ(res.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->opcua_deactivate_calls, 1);

  // cleanup
  res = node->trigger_transition(
    rclcpp_lifecycle::Transition{
    lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP});
  EXPECT_EQ(res.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
