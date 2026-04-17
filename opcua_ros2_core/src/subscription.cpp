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
#include "opcua_ros2_core/subscription.hpp"

#include <atomic>
#include <mutex>
#include <queue>

namespace opcua_ros2
{

// ---------------------------------------------------------------------------
// MonitoredItem::Impl
// ---------------------------------------------------------------------------
struct MonitoredItem::Impl
{
  NodeId   node_id;
  uint32_t server_handle{0};
  DataChangeCallback callback;
  std::mutex cb_mtx;
};

MonitoredItem::MonitoredItem(std::shared_ptr<Impl> impl)
: impl_(std::move(impl)) {}

void MonitoredItem::set_data_change_callback(DataChangeCallback cb)
{
  std::lock_guard lk(impl_->cb_mtx);
  impl_->callback = std::move(cb);
}
NodeId   MonitoredItem::node_id()       const noexcept {return impl_->node_id;}
uint32_t MonitoredItem::server_handle() const noexcept {return impl_->server_handle;}

// ---------------------------------------------------------------------------
// Subscription::Impl
// ---------------------------------------------------------------------------
struct Subscription::Impl
{
  std::vector<std::shared_ptr<MonitoredItem>> items;
  mutable std::mutex items_mtx;

  // Pending notification queue (producer = OPC UA IO thread, consumer = ROS executor)
  struct Notification { std::shared_ptr<MonitoredItem::Impl> item; DataValue dv; };
  std::queue<Notification> queue;
  std::mutex queue_mtx;
  std::atomic<bool> has_data{false};
};

Subscription::Subscription(std::shared_ptr<Impl> impl)
: impl_(std::move(impl)) {}

std::shared_ptr<MonitoredItem> Subscription::add_item(MonitoredItemConfig cfg)
{
  auto mi_impl = std::make_shared<MonitoredItem::Impl>();
  mi_impl->node_id = cfg.node_id;
  auto item = std::make_shared<MonitoredItem>(mi_impl);
  std::lock_guard lk(impl_->items_mtx);
  impl_->items.push_back(item);
  return item;
}

void Subscription::remove_item(std::shared_ptr<MonitoredItem> item)
{
  std::lock_guard lk(impl_->items_mtx);
  auto & v = impl_->items;
  v.erase(std::remove(v.begin(), v.end(), item), v.end());
}

std::size_t Subscription::item_count() const noexcept
{
  std::lock_guard lk(impl_->items_mtx);
  return impl_->items.size();
}

bool Subscription::is_ready(const rcl_wait_set_t & /*ws*/)
{
  return impl_->has_data.load(std::memory_order_acquire);
}

std::shared_ptr<void> Subscription::take_data()
{
  auto batch = std::make_shared<std::vector<std::pair<
        std::shared_ptr<MonitoredItem::Impl>, DataValue>>>();
  std::lock_guard lk(impl_->queue_mtx);
  while (!impl_->queue.empty()) {
    auto & n = impl_->queue.front();
    batch->emplace_back(n.item, std::move(n.dv));
    impl_->queue.pop();
  }
  impl_->has_data.store(false, std::memory_order_release);
  return batch;
}

void Subscription::execute(const std::shared_ptr<void> & data)
{
  using Batch = std::vector<std::pair<std::shared_ptr<MonitoredItem::Impl>, DataValue>>;
  if (!data) {return;}
  auto & batch = *std::static_pointer_cast<Batch>(data);
  for (auto & [mi, dv] : batch) {
    std::lock_guard lk(mi->cb_mtx);
    if (mi->callback) {mi->callback(dv, nullptr);}
  }
}

void Subscription::add_to_wait_set(rcl_wait_set_t & /*ws*/)
{
  // No-op: we rely on is_ready polling by the executor.
}

}  // namespace opcua_ros2
