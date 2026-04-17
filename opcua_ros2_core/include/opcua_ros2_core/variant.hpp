// Copyright 2024 Vishnuprasad Prachandabhanu
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "opcua_ros2_core/error.hpp"
#include "opcua_ros2_core/types.hpp"
#include "opcua_ros2_core/visibility_control.hpp"

#include "opcua_ros2_interfaces/msg/variant.hpp"

namespace opcua_ros2
{

/// OPC UA Variant — maps 1-to-1 with opcua_ros2_interfaces/msg/Variant.
/// Contains exactly one scalar or one array (same discriminator as the message TYPE_* constants).
class OPCUA_ROS2_CORE_PUBLIC Variant {
public:
  // Type tag constants — mirror msg::Variant::TYPE_*
  static constexpr uint8_t TYPE_NULL = 0;
  static constexpr uint8_t TYPE_BOOL = 1;
  static constexpr uint8_t TYPE_INT8 = 2;
  static constexpr uint8_t TYPE_UINT8 = 3;
  static constexpr uint8_t TYPE_INT16 = 4;
  static constexpr uint8_t TYPE_UINT16 = 5;
  static constexpr uint8_t TYPE_INT32 = 6;
  static constexpr uint8_t TYPE_UINT32 = 7;
  static constexpr uint8_t TYPE_INT64 = 8;
  static constexpr uint8_t TYPE_UINT64 = 9;
  static constexpr uint8_t TYPE_FLOAT = 10;
  static constexpr uint8_t TYPE_DOUBLE = 11;
  static constexpr uint8_t TYPE_STRING = 12;
  static constexpr uint8_t TYPE_DATETIME = 13;
  static constexpr uint8_t TYPE_GUID = 14;
  static constexpr uint8_t TYPE_BYTESTRING = 15;
  static constexpr uint8_t TYPE_NODEID = 16;
  static constexpr uint8_t TYPE_QUALIFIED_NAME = 17;
  static constexpr uint8_t TYPE_LOCALIZED_TEXT = 18;
  static constexpr uint8_t TYPE_STATUS_CODE = 19;
  static constexpr uint8_t TYPE_EXTENSION_OBJECT = 20;

  Variant() = default;

  // --- scalar factories ---
  static Variant make_bool(bool v);
  static Variant make_int8(int8_t v);
  static Variant make_uint8(uint8_t v);
  static Variant make_int16(int16_t v);
  static Variant make_uint16(uint16_t v);
  static Variant make_int32(int32_t v);
  static Variant make_uint32(uint32_t v);
  static Variant make_int64(int64_t v);
  static Variant make_uint64(uint64_t v);
  static Variant make_float(float v);
  static Variant make_double(double v);
  static Variant make_string(std::string v);
  static Variant make_node_id(NodeId v);
  static Variant make_status_code(StatusCode v);
  static Variant make_qualified_name(QualifiedName v);
  static Variant make_localized_text(LocalizedText v);
  static Variant make_guid(Guid v);
  static Variant make_bytestring(std::vector<uint8_t> v);

  // --- array factories ---
  static Variant make_bool_array(std::vector<uint8_t> v, std::vector<int32_t> dims = {});
  static Variant make_int_array(std::vector<int64_t> v, std::vector<int32_t> dims = {});
  static Variant make_uint_array(std::vector<uint64_t> v, std::vector<int32_t> dims = {});
  static Variant make_double_array(std::vector<double> v, std::vector<int32_t> dims = {});
  static Variant make_string_array(std::vector<std::string> v, std::vector<int32_t> dims = {});

  bool is_null()   const noexcept {return type_ == TYPE_NULL;}
  bool is_scalar() const noexcept {return !is_null() && !is_array_;}
  bool is_array()  const noexcept {return is_array_;}
  std::optional<uint8_t> type_tag() const noexcept
  {
    if (is_null()) {return std::nullopt;}
    return type_;
  }

  // --- scalar getters (throw Error on type mismatch) ---
  bool             get_bool()          const;
  int64_t          get_int()           const;
  uint64_t         get_uint()          const;
  double           get_double()        const;
  const std::string & get_string()      const;
  const NodeId & get_node_id()       const;
  const StatusCode & get_status_code()  const;
  const QualifiedName & get_qualified_name() const;
  const LocalizedText & get_localized_text() const;
  const Guid & get_guid()          const;
  const std::vector<uint8_t> & get_bytestring() const;

  // --- array getters ---
  std::span<const uint8_t> get_bool_array()   const;
  std::span<const int64_t> get_int_array()    const;
  std::span<const uint64_t> get_uint_array()   const;
  std::span<const double> get_double_array() const;
  std::span<const std::string> get_string_array() const;

  const std::vector<int32_t> & array_dimensions() const noexcept {return dims_;}

  // --- ROS message round-trip (implemented in variant.cpp, no open62541 dependency) ---
  static Variant from_ros(const opcua_ros2_interfaces::msg::Variant & msg);
  opcua_ros2_interfaces::msg::Variant to_ros() const;

private:
  uint8_t type_{TYPE_NULL};
  bool    is_array_{false};
  std::vector<int32_t> dims_;

  // Scalar storage
  bool            bool_val_{false};
  int64_t         int_val_{0};
  uint64_t        uint_val_{0};
  double          double_val_{0.0};
  std::string     string_val_;
  NodeId          node_id_val_;
  StatusCode      status_val_;
  QualifiedName   qname_val_;
  LocalizedText   ltext_val_;
  Guid            guid_val_{};
  std::vector<uint8_t> bytestring_val_;

  // Array storage (bool stored as uint8_t to allow span)
  std::vector<uint8_t> bool_arr_;
  std::vector<int64_t> int_arr_;
  std::vector<uint64_t> uint_arr_;
  std::vector<double> double_arr_;
  std::vector<std::string> string_arr_;
};

}  // namespace opcua_ros2
