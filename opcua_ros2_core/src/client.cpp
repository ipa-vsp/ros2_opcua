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
#include "opcua_ros2_core/client.hpp"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>

// open62541pp — only in this TU, never leaks to public headers.
#include <open62541pp/client.hpp>
#include <open62541pp/config.hpp>
#include <open62541pp/services/attribute_highlevel.hpp>
#include <open62541pp/services/method.hpp>
#include <open62541pp/services/view.hpp>
#include <open62541pp/types.hpp>
#include <open62541pp/ua/types.hpp>

#include "opcua_ros2_core/detail/ua_conversion.hpp"

namespace opcua_ros2
{

// ---------------------------------------------------------------------------
// Impl
// ---------------------------------------------------------------------------
struct Client::Impl
{
  explicit Impl(ClientConfig cfg)
  : config_(std::move(cfg)) {}

  ~Impl()
  {
    do_stop();
  }

  tl::expected<void, Error> connect()
  {
    if (connected_.load()) {return {};}
    try {
      opcua::ClientConfig cc;
      if (!config_.username.empty()) {
        cc.setUserIdentityToken(
          opcua::UserNameIdentityToken{config_.username, config_.password});
      }
      client_ = std::make_unique<opcua::Client>(std::move(cc));

      client_->onDisconnected([this]() {
          connected_.store(false, std::memory_order_relaxed);
      });
      client_->onConnected([this]() {
          connected_.store(true, std::memory_order_relaxed);
      });

      client_->connect(config_.endpoint_url);
      connected_.store(true, std::memory_order_relaxed);

      // Start the run-loop thread.
      stop_flag_.store(false, std::memory_order_relaxed);
      io_thread_ = std::thread([this]() {io_loop();});
      return {};
    } catch (const std::exception & ex) {
      return tl::unexpected(Error::from_status(StatusCode::BadUnexpected, ex.what()));
    }
  }

  tl::expected<void, Error> disconnect()
  {
    do_stop();
    return {};
  }

  bool is_connected() const noexcept
  {
    return connected_.load(std::memory_order_relaxed);
  }

  tl::expected<DataValue, Error> read(const NodeId & id, uint32_t /*attr*/)
  {
    return with_client([&](opcua::Client & c) -> tl::expected<DataValue, Error> {
               auto res = opcua::services::readValue(c, detail::to_ua(id));
               if (!res.hasValue()) {
                 return tl::unexpected(
          Error::from_status(static_cast<uint32_t>(res.code()), "readValue failed"));
               }
               DataValue dv;
               dv.value = detail::to_core_variant(*res);
               return dv;
    });
  }

  tl::expected<void, Error> write(const NodeId & id, const DataValue & dv, uint32_t /*attr*/)
  {
    return with_client([&](opcua::Client & c) -> tl::expected<void, Error> {
               auto ua_v = detail::to_ua_variant(dv.value);
               auto sc = opcua::services::writeValue(c, detail::to_ua(id), ua_v);
               if (sc.isBad()) {
                 return tl::unexpected(
          Error::from_status(static_cast<uint32_t>(sc), "writeValue failed"));
               }
               return {};
    });
  }

  tl::expected<BrowseResult, Error> browse(const BrowseRequest & req)
  {
    return with_client([&](opcua::Client & c) -> tl::expected<BrowseResult, Error> {
               bool has_ref_type =
               !(req.reference_type_id.namespace_index() == 0 &&
               req.reference_type_id.identifier_type() == NodeId::IdentifierType::Numeric &&
               req.reference_type_id.numeric_id() == 0);

               opcua::BrowseDescription bd{
                 detail::to_ua(req.node_id),
                 static_cast<opcua::BrowseDirection>(req.browse_direction),
                 has_ref_type ? detail::to_ua(req.reference_type_id) : opcua::NodeId{},
                 req.include_subtypes
               };

               auto ua_result = opcua::services::browse(c, bd, 0);

               BrowseResult result;
               for (const auto & r : ua_result.references()) {
                 BrowseResultEntry e;
                 e.node_id = detail::to_core(r.nodeId().nodeId());
                 e.browse_name = std::string{r.browseName().name()};
                 e.display_name = std::string{r.displayName().text()};
                 e.node_class = static_cast<uint32_t>(r.nodeClass());
                 e.reference_type_id = detail::to_core(r.referenceTypeId());
                 e.is_forward = r.isForward();
                 result.push_back(std::move(e));
               }
               return result;
    });
  }

  tl::expected<CallResult, Error> call(
    const NodeId & object_id, const NodeId & method_id,
    std::span<const Variant> inputs)
  {
    return with_client([&](opcua::Client & c) -> tl::expected<CallResult, Error> {
               std::vector<opcua::Variant> ua_inputs;
               ua_inputs.reserve(inputs.size());
               for (const auto & v : inputs) {ua_inputs.push_back(detail::to_ua_variant(v));}

               auto ua_result = opcua::services::call(
        c,
        detail::to_ua(object_id),
        detail::to_ua(method_id),
        opcua::Span<const opcua::Variant>{ua_inputs.data(), ua_inputs.size()});

               CallResult result;
               for (const auto & ov : ua_result.outputArguments()) {
                 result.push_back(detail::to_core_variant(ov));
               }
               return result;
    });
  }

  std::future<tl::expected<DataValue, Error>> async_read(const NodeId & id, uint32_t attr)
  {
    auto promise = std::make_shared<std::promise<tl::expected<DataValue, Error>>>();
    auto future = promise->get_future();
    std::thread([this, id, attr, p = std::move(promise)]() mutable {
        p->set_value(read(id, attr));
    }).detach();
    return future;
  }

  std::future<tl::expected<void, Error>> async_write(
    const NodeId & id,
    const DataValue & dv,
    uint32_t attr)
  {
    auto promise = std::make_shared<std::promise<tl::expected<void, Error>>>();
    auto future = promise->get_future();
    std::thread([this, id, dv, attr, p = std::move(promise)]() mutable {
        p->set_value(write(id, dv, attr));
    }).detach();
    return future;
  }

  uint64_t session_id() const noexcept {return 0;}
  const ClientConfig & config() const noexcept {return config_;}

private:
  /// Execute fn on the client thread. The IO loop pauses to allow the call.
  template<typename Fn>
  auto with_client(Fn && fn) -> decltype(fn(std::declval<opcua::Client &>()))
  {
    if (!client_) {
      return tl::unexpected(Error::from_status(StatusCode::BadDisconnect, "not connected"));
    }

    std::unique_lock lk(op_mtx_);
    pending_op_ = true;
    op_cv_.notify_one();
    // Wait until the IO thread sees pending_op_ and grants us the client
    op_done_cv_.wait(lk, [this] {return io_idle_.load();});
    auto result = fn(*client_);
    pending_op_ = false;
    io_idle_.store(false);
    lk.unlock();
    op_cv_.notify_one();  // release IO thread
    return result;
  }

  void io_loop()
  {
    while (!stop_flag_.load(std::memory_order_relaxed)) {
      {
        std::unique_lock lk(op_mtx_);
        if (pending_op_) {
          // Signal that we're paused
          io_idle_.store(true);
          op_done_cv_.notify_one();
          // Wait for the operation to complete
          op_cv_.wait(lk, [this] {return !pending_op_ || stop_flag_.load();});
          io_idle_.store(false);
        }
      }
      if (stop_flag_.load()) {break;}
      client_->runIterate(5);
    }
  }

  void do_stop()
  {
    stop_flag_.store(true, std::memory_order_relaxed);
    {
      std::lock_guard lk(op_mtx_);
      pending_op_ = false;
    }
    op_cv_.notify_all();
    op_done_cv_.notify_all();
    if (io_thread_.joinable()) {io_thread_.join();}
    if (client_) {
      try {client_->disconnect();} catch (...) {}
      client_.reset();
    }
    connected_.store(false, std::memory_order_relaxed);
  }

  ClientConfig config_;
  std::unique_ptr<opcua::Client> client_;
  std::atomic<bool> connected_{false};
  std::atomic<bool> stop_flag_{true};
  std::thread io_thread_;

  // Synchronization for exclusive client access
  std::mutex              op_mtx_;
  std::condition_variable op_cv_;
  std::condition_variable op_done_cv_;
  std::atomic<bool> io_idle_{false};
  bool                    pending_op_{false};
};

// ---------------------------------------------------------------------------
// Client delegation
// ---------------------------------------------------------------------------
Client::Client(ClientConfig cfg)
: impl_(std::make_unique<Impl>(std::move(cfg))) {}
Client::~Client() = default;

tl::expected<void, Error> Client::connect() {return impl_->connect();}
tl::expected<void, Error> Client::disconnect() {return impl_->disconnect();}
bool                       Client::is_connected() const noexcept {return impl_->is_connected();}

tl::expected<DataValue, Error> Client::read(const NodeId & id, uint32_t attr)
{
  return impl_->read(id, attr);
}
tl::expected<void, Error> Client::write(const NodeId & id, const DataValue & dv, uint32_t attr)
{
  return impl_->write(id, dv, attr);
}
tl::expected<BrowseResult, Error> Client::browse(const BrowseRequest & req)
{
  return impl_->browse(req);
}
tl::expected<CallResult, Error> Client::call(
  const NodeId & obj, const NodeId & meth,
  std::span<const Variant> inputs)
{
  return impl_->call(obj, meth, inputs);
}
std::future<tl::expected<DataValue, Error>> Client::async_read(const NodeId & id, uint32_t attr)
{
  return impl_->async_read(id, attr);
}
std::future<tl::expected<void, Error>> Client::async_write(
  const NodeId & id,
  const DataValue & dv,
  uint32_t attr)
{
  return impl_->async_write(id, dv, attr);
}
std::shared_ptr<opcua_ros2::Subscription> Client::create_subscription(
  opcua_ros2::SubscriptionConfig /*cfg*/,
  rclcpp::CallbackGroup::SharedPtr /*cb*/)
{
  throw std::runtime_error("create_subscription not yet implemented");
}
const ClientConfig & Client::config() const noexcept {return impl_->config();}
uint64_t             Client::session_id() const noexcept {return impl_->session_id();}

}  // namespace opcua_ros2
