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
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "opcua_ros2_core/client.hpp"
#include "opcua_ros2_core/server.hpp"
#include "opcua_ros2_core/variant.hpp"

using namespace opcua_ros2;
using namespace std::chrono_literals;

class ClientServerFixture : public ::testing::Test {
protected:
  void SetUp() override
  {
    ServerConfig sc;
    sc.port = 14840;  // non-default port to avoid conflicts
    server_ = std::make_unique<Server>(sc);
    ASSERT_TRUE(server_->start().has_value()) << "server start failed";
    std::this_thread::sleep_for(100ms);  // let server accept connections
  }

  void TearDown() override
  {
    if (client_ && client_->is_connected()) {client_->disconnect();}
    server_->stop();
    std::this_thread::sleep_for(50ms);
  }

  std::unique_ptr<Server> server_;
  std::unique_ptr<Client> client_;
};

TEST_F(ClientServerFixture, ConnectDisconnect) {
  ClientConfig cc;
  cc.endpoint_url = "opc.tcp://localhost:14840";
  client_ = std::make_unique<Client>(cc);

  auto res = client_->connect();
  EXPECT_TRUE(res.has_value()) << "connect error: " << (res ? "" : res.error().message);
  EXPECT_TRUE(client_->is_connected());

  auto dc = client_->disconnect();
  EXPECT_TRUE(dc.has_value());
  EXPECT_FALSE(client_->is_connected());
}

TEST_F(ClientServerFixture, BrowseRootNode) {
  ClientConfig cc;
  cc.endpoint_url = "opc.tcp://localhost:14840";
  client_ = std::make_unique<Client>(cc);
  ASSERT_TRUE(client_->connect().has_value());

  BrowseRequest req;
  req.node_id = NodeId{0, uint32_t{84}};  // ns=0;i=84 = Root

  auto res = client_->browse(req);
  EXPECT_TRUE(res.has_value()) << (res ? "" : res.error().message);
  if (res) {
    EXPECT_GT(res->size(), 0u) << "root node should have children";
  }
}

TEST_F(ClientServerFixture, AsyncReadServerTime) {
  ClientConfig cc;
  cc.endpoint_url = "opc.tcp://localhost:14840";
  client_ = std::make_unique<Client>(cc);
  ASSERT_TRUE(client_->connect().has_value());

  // CurrentTime node: ns=0;i=2258
  NodeId time_node{0, uint32_t{2258}};
  auto fut = client_->async_read(time_node);
  auto status = fut.wait_for(2s);
  ASSERT_EQ(status, std::future_status::ready);
  auto res = fut.get();
  EXPECT_TRUE(res.has_value()) << (res ? "" : res.error().message);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
