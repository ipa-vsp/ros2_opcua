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
#include "opcua_ros2_core/server.hpp"

#include <atomic>
#include <stdexcept>
#include <thread>

// open62541pp — only in this TU.
#include <open62541pp/server.hpp>
#include <open62541pp/services/attribute_highlevel.hpp>

#include "opcua_ros2_core/detail/ua_conversion.hpp"

namespace opcua_ros2
{

struct Server::Impl
{
  explicit Impl(ServerConfig cfg)
  : config_(std::move(cfg)) {}
  ~Impl() {do_stop();}

  tl::expected<void, Error> start()
  {
    if (running_.load()) {return {};}
    try {
      opcua::ServerConfig sc{config_.port};
      server_ = std::make_unique<opcua::Server>(std::move(sc));
      running_.store(true);
      io_thread_ = std::thread([this]() {
            server_->run();
            running_.store(false);
      });
      return {};
    } catch (const std::exception & ex) {
      return tl::unexpected(Error::from_status(StatusCode::BadUnexpected, ex.what()));
    }
  }

  tl::expected<void, Error> stop()
  {
    do_stop();
    return {};
  }

  bool is_running() const noexcept {return running_.load();}
  uint16_t port() const noexcept {return config_.port;}

  tl::expected<void, Error> write_variable(const NodeId & id, const DataValue & dv)
  {
    if (!server_) {
      return tl::unexpected(Error::from_status(StatusCode::BadUnexpected, "server not started"));
    }
    try {
      auto ua_v = detail::to_ua_variant(dv.value);
      auto sc = opcua::services::writeValue(*server_, detail::to_ua(id), ua_v);
      if (sc.isBad()) {
        return tl::unexpected(Error::from_status(static_cast<uint32_t>(sc), "writeValue failed"));
      }
      return {};
    } catch (const std::exception & ex) {
      return tl::unexpected(Error::from_status(StatusCode::BadUnexpected, ex.what()));
    }
  }

  tl::expected<DataValue, Error> read_variable(const NodeId & id)
  {
    if (!server_) {
      return tl::unexpected(Error::from_status(StatusCode::BadUnexpected, "server not started"));
    }
    try {
      auto res = opcua::services::readValue(*server_, detail::to_ua(id));
      if (!res.hasValue()) {
        return tl::unexpected(
          Error::from_status(static_cast<uint32_t>(res.code()), "readValue failed"));
      }
      DataValue dv;
      dv.value = detail::to_core_variant(*res);
      return dv;
    } catch (const std::exception & ex) {
      return tl::unexpected(Error::from_status(StatusCode::BadUnexpected, ex.what()));
    }
  }

private:
  void do_stop()
  {
    if (!running_.load()) {return;}
    if (server_) {server_->stop();}
    if (io_thread_.joinable()) {io_thread_.join();}
    server_.reset();
  }

  ServerConfig config_;
  std::unique_ptr<opcua::Server> server_;
  std::atomic<bool> running_{false};
  std::thread io_thread_;
};

Server::Server(ServerConfig cfg)
: impl_(std::make_unique<Impl>(std::move(cfg))) {}
Server::~Server() = default;

tl::expected<void, Error> Server::start() {return impl_->start();}
tl::expected<void, Error> Server::stop() {return impl_->stop();}
bool                       Server::is_running() const noexcept {return impl_->is_running();}
tl::expected<void, Error> Server::write_variable(const NodeId & id, const DataValue & dv)
{return impl_->write_variable(id, dv);}
tl::expected<DataValue, Error> Server::read_variable(const NodeId & id)
{return impl_->read_variable(id);}
uint16_t Server::port() const noexcept {return impl_->port();}

}  // namespace opcua_ros2
