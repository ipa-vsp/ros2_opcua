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
#include "opcua_ros2_core/type_registry.hpp"

#include <map>
#include <mutex>
#include <string>

namespace opcua_ros2
{

struct TypeRegistry::Impl
{
  struct Entry
  {
    std::string ros_type_name;
    Encoder encoder;
    Decoder decoder;
  };
  std::map<std::string, Entry> codecs;  // keyed by NodeId::to_string()
  mutable std::mutex mtx;
};

TypeRegistry::TypeRegistry()
: impl_(std::make_unique<Impl>()) {}
TypeRegistry::~TypeRegistry() = default;

TypeRegistry & TypeRegistry::instance()
{
  static TypeRegistry inst;
  return inst;
}

void TypeRegistry::register_codec(
  const NodeId & id, std::string_view ros_type_name,
  Encoder encoder, Decoder decoder)
{
  std::lock_guard lk(impl_->mtx);
  impl_->codecs[id.to_string()] = Impl::Entry{std::string{ros_type_name},
    std::move(encoder), std::move(decoder)};
}

std::optional<Encoder> TypeRegistry::find_encoder(const NodeId & id) const
{
  std::lock_guard lk(impl_->mtx);
  auto it = impl_->codecs.find(id.to_string());
  if (it == impl_->codecs.end()) {return std::nullopt;}
  return it->second.encoder;
}

std::optional<Decoder> TypeRegistry::find_decoder(const NodeId & id) const
{
  std::lock_guard lk(impl_->mtx);
  auto it = impl_->codecs.find(id.to_string());
  if (it == impl_->codecs.end()) {return std::nullopt;}
  return it->second.decoder;
}

void TypeRegistry::clear()
{
  std::lock_guard lk(impl_->mtx);
  impl_->codecs.clear();
}

}  // namespace opcua_ros2
