// Copyright 2024 Vishnuprasad Prachandabhanu
// SPDX-License-Identifier: Apache-2.0
#include "opcua_ros2_core/node_id.hpp"

#include <charconv>
#include <sstream>
#include <stdexcept>

namespace opcua_ros2
{

NodeId::NodeId(uint16_t ns, uint32_t numeric) noexcept
: ns_(ns), id_(numeric) {}
NodeId::NodeId(uint16_t ns, std::string s) noexcept
: ns_(ns), id_(std::move(s)) {}
NodeId::NodeId(uint16_t ns, opcua_ros2::Guid guid) noexcept
: ns_(ns), id_(guid) {}
NodeId::NodeId(uint16_t ns, std::vector<uint8_t> opaque) noexcept
: ns_(ns), id_(std::move(opaque)) {}

uint16_t NodeId::namespace_index() const noexcept {return ns_;}

NodeId::IdentifierType NodeId::identifier_type() const noexcept
{
  return static_cast<IdentifierType>(id_.index());
}

uint32_t NodeId::numeric_id() const
{
  if (auto * p = std::get_if<uint32_t>(&id_)) {return *p;}
  throw std::runtime_error("NodeId is not numeric");
}
const std::string & NodeId::string_id() const
{
  if (auto * p = std::get_if<std::string>(&id_)) {return *p;}
  throw std::runtime_error("NodeId is not a string");
}
const opcua_ros2::Guid & NodeId::guid_id() const
{
  if (auto * p = std::get_if<opcua_ros2::Guid>(&id_)) {return *p;}
  throw std::runtime_error("NodeId is not a GUID");
}
const std::vector<uint8_t> & NodeId::opaque_id() const
{
  if (auto * p = std::get_if<std::vector<uint8_t>>(&id_)) {return *p;}
  throw std::runtime_error("NodeId is not opaque");
}

std::string NodeId::to_string() const
{
  std::ostringstream oss;
  if (ns_ != 0) {oss << "ns=" << ns_ << ";";}
  std::visit([&](auto && arg) {
      using T = std::decay_t<decltype(arg)>;
      if constexpr (std::is_same_v<T, uint32_t>) {
        oss << "i=" << arg;
      } else if constexpr (std::is_same_v<T, std::string>) {
        oss << "s=" << arg;
      } else if constexpr (std::is_same_v<T, Guid>) {
        oss << "g=";
        for (std::size_t i = 0; i < 16; ++i) {
          if (i == 4 || i == 6 || i == 8 || i == 10) {oss << '-';}
          oss << std::hex << static_cast<int>(arg[i]);
        }
      } else {
        oss << "b=<opaque>";
      }
  }, id_);
  return oss.str();
}

tl::expected<NodeId, Error> NodeId::parse(std::string_view sv) noexcept
{
  uint16_t ns = 0;
  std::string_view rest = sv;

  // Optional ns=X; prefix
  if (sv.starts_with("ns=")) {
    auto semi = sv.find(';');
    if (semi == std::string_view::npos) {
      return tl::unexpected(Error::from_status(StatusCode::BadUnexpected, "Missing ';' in NodeId"));
    }
    auto ns_str = sv.substr(3, semi - 3);
    auto [ptr, ec] = std::from_chars(ns_str.data(), ns_str.data() + ns_str.size(), ns);
    if (ec != std::errc{}) {
      return tl::unexpected(Error::from_status(StatusCode::BadUnexpected, "Bad ns index"));
    }
    rest = sv.substr(semi + 1);
  }

  if (rest.starts_with("i=")) {
    uint32_t num = 0;
    auto [ptr, ec] = std::from_chars(rest.data() + 2, rest.data() + rest.size(), num);
    if (ec != std::errc{}) {
      return tl::unexpected(Error::from_status(StatusCode::BadUnexpected, "Bad numeric id"));
    }
    return NodeId{ns, num};
  } else if (rest.starts_with("s=")) {
    return NodeId{ns, std::string{rest.substr(2)}};
  }
  return tl::unexpected(Error::from_status(StatusCode::BadUnexpected, "Unsupported NodeId format"));
}

}  // namespace opcua_ros2
