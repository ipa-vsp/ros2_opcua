// Copyright 2024 Vishnuprasad Prachandabhanu
// SPDX-License-Identifier: Apache-2.0
#include "opcua_ros2_core/variant.hpp"

#include <stdexcept>

#include "opcua_ros2_interfaces/msg/variant.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

namespace opcua_ros2
{

namespace
{
void check_scalar(uint8_t actual, uint8_t expected)
{
  if (actual != expected) {
    throw std::runtime_error("Variant type mismatch");
  }
}
void check_array(bool is_array)
{
  if (!is_array) {throw std::runtime_error("Variant is not an array");}
}
}  // namespace

// --- scalar factories ---
Variant Variant::make_bool(bool v) {Variant r; r.type_ = TYPE_BOOL;   r.bool_val_ = v;   return r;}
Variant Variant::make_int8(int8_t v)
{
  Variant r; r.type_ = TYPE_INT8;   r.int_val_ = v;    return r;
}
Variant Variant::make_uint8(uint8_t v)
{
  Variant r; r.type_ = TYPE_UINT8;  r.uint_val_ = v;   return r;
}
Variant Variant::make_int16(int16_t v)
{
  Variant r; r.type_ = TYPE_INT16;  r.int_val_ = v;    return r;
}
Variant Variant::make_uint16(uint16_t v)
{
  Variant r; r.type_ = TYPE_UINT16; r.uint_val_ = v;   return r;
}
Variant Variant::make_int32(int32_t v)
{
  Variant r; r.type_ = TYPE_INT32;  r.int_val_ = v;    return r;
}
Variant Variant::make_uint32(uint32_t v)
{
  Variant r; r.type_ = TYPE_UINT32; r.uint_val_ = v;   return r;
}
Variant Variant::make_int64(int64_t v)
{
  Variant r; r.type_ = TYPE_INT64;  r.int_val_ = v;    return r;
}
Variant Variant::make_uint64(uint64_t v)
{
  Variant r; r.type_ = TYPE_UINT64; r.uint_val_ = v;   return r;
}
Variant Variant::make_float(float v)
{
  Variant r; r.type_ = TYPE_FLOAT;  r.double_val_ = static_cast<double>(v); return r;
}
Variant Variant::make_double(double v)
{
  Variant r; r.type_ = TYPE_DOUBLE; r.double_val_ = v; return r;
}
Variant Variant::make_string(std::string v)
{
  Variant r; r.type_ = TYPE_STRING; r.string_val_ = std::move(v); return r;
}
Variant Variant::make_node_id(NodeId v)
{
  Variant r; r.type_ = TYPE_NODEID; r.node_id_val_ = std::move(v); return r;
}
Variant Variant::make_status_code(StatusCode v)
{
  Variant r; r.type_ = TYPE_STATUS_CODE; r.status_val_ = v; return r;
}
Variant Variant::make_qualified_name(QualifiedName v)
{
  Variant r; r.type_ = TYPE_QUALIFIED_NAME; r.qname_val_ = std::move(v); return r;
}
Variant Variant::make_localized_text(LocalizedText v)
{
  Variant r; r.type_ = TYPE_LOCALIZED_TEXT; r.ltext_val_ = std::move(v); return r;
}
Variant Variant::make_guid(Guid v) {Variant r; r.type_ = TYPE_GUID; r.guid_val_ = v; return r;}
Variant Variant::make_bytestring(std::vector<uint8_t> v)
{
  Variant r; r.type_ = TYPE_BYTESTRING; r.bytestring_val_ = std::move(v); return r;
}

// --- array factories ---
Variant Variant::make_bool_array(std::vector<uint8_t> v, std::vector<int32_t> dims)
{
  Variant r; r.type_ = TYPE_BOOL;   r.is_array_ = true; r.bool_arr_ = std::move(v);
  r.dims_ = std::move(dims); return r;
}
Variant Variant::make_int_array(std::vector<int64_t> v, std::vector<int32_t> dims)
{
  Variant r; r.type_ = TYPE_INT64;  r.is_array_ = true; r.int_arr_ = std::move(v);
  r.dims_ = std::move(dims); return r;
}
Variant Variant::make_uint_array(std::vector<uint64_t> v, std::vector<int32_t> dims)
{
  Variant r; r.type_ = TYPE_UINT64; r.is_array_ = true; r.uint_arr_ = std::move(v);
  r.dims_ = std::move(dims); return r;
}
Variant Variant::make_double_array(std::vector<double> v, std::vector<int32_t> dims)
{
  Variant r; r.type_ = TYPE_DOUBLE; r.is_array_ = true; r.double_arr_ = std::move(v);
  r.dims_ = std::move(dims); return r;
}
Variant Variant::make_string_array(std::vector<std::string> v, std::vector<int32_t> dims)
{
  Variant r; r.type_ = TYPE_STRING; r.is_array_ = true; r.string_arr_ = std::move(v);
  r.dims_ = std::move(dims); return r;
}

// --- scalar getters ---
bool              Variant::get_bool()    const {check_scalar(type_, TYPE_BOOL);   return bool_val_;}
int64_t           Variant::get_int()     const
{
  if (type_ == TYPE_INT8 || type_ == TYPE_INT16 || type_ == TYPE_INT32 || type_ == TYPE_INT64) {
    return int_val_;
  }
  throw std::runtime_error("Variant not an int type");
}
uint64_t          Variant::get_uint()    const
{
  if (type_ == TYPE_UINT8 || type_ == TYPE_UINT16 || type_ == TYPE_UINT32 || type_ == TYPE_UINT64) {
    return uint_val_;
  }
  throw std::runtime_error("Variant not a uint type");
}
double            Variant::get_double()  const
{
  if (type_ == TYPE_FLOAT || type_ == TYPE_DOUBLE) {return double_val_;}
  throw std::runtime_error("Variant not a float/double type");
}
const std::string & Variant::get_string() const
{
  check_scalar(type_, TYPE_STRING); return string_val_;
}
const NodeId & Variant::get_node_id() const {check_scalar(type_, TYPE_NODEID); return node_id_val_;}
const StatusCode & Variant::get_status_code() const
{
  check_scalar(type_, TYPE_STATUS_CODE); return status_val_;
}
const QualifiedName & Variant::get_qualified_name() const
{
  check_scalar(type_, TYPE_QUALIFIED_NAME); return qname_val_;
}
const LocalizedText & Variant::get_localized_text() const
{
  check_scalar(type_, TYPE_LOCALIZED_TEXT); return ltext_val_;
}
const Guid & Variant::get_guid()       const {check_scalar(type_, TYPE_GUID); return guid_val_;}
const std::vector<uint8_t> & Variant::get_bytestring() const
{
  check_scalar(type_, TYPE_BYTESTRING); return bytestring_val_;
}

// --- array getters ---
std::span<const uint8_t> Variant::get_bool_array()   const
{
  check_array(is_array_); return bool_arr_;
}
std::span<const int64_t> Variant::get_int_array()    const
{
  check_array(is_array_); return int_arr_;
}
std::span<const uint64_t> Variant::get_uint_array()   const
{
  check_array(is_array_); return uint_arr_;
}
std::span<const double> Variant::get_double_array() const
{
  check_array(is_array_); return double_arr_;
}
std::span<const std::string> Variant::get_string_array() const
{
  check_array(is_array_); return string_arr_;
}

// --- ROS message conversion ---
Variant Variant::from_ros(const opcua_ros2_interfaces::msg::Variant & msg)
{
  Variant v;
  v.type_ = msg.type;
  v.is_array_ = msg.is_array;
  v.dims_.assign(msg.array_dimensions.begin(), msg.array_dimensions.end());
  if (!msg.is_array) {
    switch (msg.type) {
      case TYPE_BOOL:            v.bool_val_ = msg.bool_value;   break;
      case TYPE_INT8:
      case TYPE_INT16:
      case TYPE_INT32:
      case TYPE_INT64:           v.int_val_ = msg.int_value;    break;
      case TYPE_UINT8:
      case TYPE_UINT16:
      case TYPE_UINT32:
      case TYPE_UINT64:          v.uint_val_ = msg.uint_value;   break;
      case TYPE_FLOAT:
      case TYPE_DOUBLE:          v.double_val_ = msg.double_value; break;
      case TYPE_STRING:          v.string_val_ = msg.string_value; break;
      case TYPE_NODEID: {
          const auto & nid = msg.nodeid_value;
          using NT = decltype(nid.identifier_type);
          if (nid.identifier_type == 0 /* numeric */) {
            v.node_id_val_ = NodeId{nid.namespace_index, nid.numeric_id}
          } else if (nid.identifier_type == 1 /* string */) {
            v.node_id_val_ = NodeId{nid.namespace_index, nid.string_id}
          } else if (nid.identifier_type == 2 /* guid */) {
            Guid g;
            std::copy(nid.guid_id.uuid.begin(), nid.guid_id.uuid.end(), g.begin());
            v.node_id_val_ = NodeId{nid.namespace_index, g};
          } else {
            v.node_id_val_ = NodeId{nid.namespace_index,
              std::vector<uint8_t>(nid.opaque_id.begin(), nid.opaque_id.end())};
          }
          break;
        }
      case TYPE_STATUS_CODE:
        v.status_val_.value = msg.status_code_value.value;
        v.status_val_.name = msg.status_code_value.name;
        break;
      case TYPE_QUALIFIED_NAME:
        v.qname_val_.namespace_index = msg.qualified_name_value.namespace_index;
        v.qname_val_.name = msg.qualified_name_value.name;
        break;
      case TYPE_LOCALIZED_TEXT:
        v.ltext_val_.locale = msg.localized_text_value.locale;
        v.ltext_val_.text = msg.localized_text_value.text;
        break;
      case TYPE_GUID:
        std::copy(msg.guid_value.uuid.begin(), msg.guid_value.uuid.end(), v.guid_val_.begin());
        break;
      case TYPE_BYTESTRING:
        v.bytestring_val_.assign(msg.bytestring_value.begin(), msg.bytestring_value.end());
        break;
      default: break;
    }
  } else {
    // Array types
    switch (msg.type) {
      case TYPE_BOOL:
        v.bool_arr_.resize(msg.bool_array.size());
        for (std::size_t i = 0; i < msg.bool_array.size(); ++i) {
          v.bool_arr_[i] = msg.bool_array[i] ? 1 : 0;
        }
        break;
      case TYPE_INT8: case TYPE_INT16: case TYPE_INT32: case TYPE_INT64:
        v.int_arr_.assign(msg.int_array.begin(), msg.int_array.end()); break;
      case TYPE_UINT8: case TYPE_UINT16: case TYPE_UINT32: case TYPE_UINT64:
        v.uint_arr_.assign(msg.uint_array.begin(), msg.uint_array.end()); break;
      case TYPE_FLOAT: case TYPE_DOUBLE:
        v.double_arr_.assign(msg.double_array.begin(), msg.double_array.end()); break;
      case TYPE_STRING:
        v.string_arr_.assign(msg.string_array.begin(), msg.string_array.end()); break;
      default: break;
    }
  }
  return v;
}

opcua_ros2_interfaces::msg::Variant Variant::to_ros() const
{
  opcua_ros2_interfaces::msg::Variant msg;
  msg.type = type_;
  msg.is_array = is_array_;
  msg.array_dimensions.assign(dims_.begin(), dims_.end());
  if (!is_array_) {
    switch (type_) {
      case TYPE_BOOL:   msg.bool_value = bool_val_;   break;
      case TYPE_INT8:
      case TYPE_INT16:
      case TYPE_INT32:
      case TYPE_INT64:  msg.int_value = int_val_;    break;
      case TYPE_UINT8:
      case TYPE_UINT16:
      case TYPE_UINT32:
      case TYPE_UINT64: msg.uint_value = uint_val_;   break;
      case TYPE_FLOAT:
      case TYPE_DOUBLE: msg.double_value = double_val_; break;
      case TYPE_STRING: msg.string_value = string_val_; break;
      case TYPE_NODEID: {
          auto & nid = msg.nodeid_value;
          nid.namespace_index = node_id_val_.namespace_index();
          nid.identifier_type = static_cast<uint8_t>(node_id_val_.identifier_type());
          switch (node_id_val_.identifier_type()) {
            case NodeId::IdentifierType::Numeric:
              nid.numeric_id = node_id_val_.numeric_id(); break;
            case NodeId::IdentifierType::String:
              nid.string_id = node_id_val_.string_id();  break;
            case NodeId::IdentifierType::Guid: {
                const auto & g = node_id_val_.guid_id();
                std::copy(g.begin(), g.end(), nid.guid_id.uuid.begin());
                break;
              }
            case NodeId::IdentifierType::Opaque: {
                const auto & op = node_id_val_.opaque_id();
                nid.opaque_id.assign(op.begin(), op.end());
                break;
              }
          }
          break;
        }
      case TYPE_STATUS_CODE:
        msg.status_code_value.value = status_val_.value;
        msg.status_code_value.name = status_val_.name;
        break;
      case TYPE_QUALIFIED_NAME:
        msg.qualified_name_value.namespace_index = qname_val_.namespace_index;
        msg.qualified_name_value.name = qname_val_.name;
        break;
      case TYPE_LOCALIZED_TEXT:
        msg.localized_text_value.locale = ltext_val_.locale;
        msg.localized_text_value.text = ltext_val_.text;
        break;
      case TYPE_GUID:
        std::copy(guid_val_.begin(), guid_val_.end(), msg.guid_value.uuid.begin());
        break;
      case TYPE_BYTESTRING:
        msg.bytestring_value.assign(bytestring_val_.begin(), bytestring_val_.end());
        break;
      default: break;
    }
  } else {
    switch (type_) {
      case TYPE_BOOL:
        msg.bool_array.resize(bool_arr_.size());
        for (std::size_t i = 0; i < bool_arr_.size(); ++i) {
          msg.bool_array[i] = bool_arr_[i] != 0;
        }
        break;
      case TYPE_INT8: case TYPE_INT16: case TYPE_INT32: case TYPE_INT64:
        msg.int_array.assign(int_arr_.begin(), int_arr_.end()); break;
      case TYPE_UINT8: case TYPE_UINT16: case TYPE_UINT32: case TYPE_UINT64:
        msg.uint_array.assign(uint_arr_.begin(), uint_arr_.end()); break;
      case TYPE_FLOAT: case TYPE_DOUBLE:
        msg.double_array.assign(double_arr_.begin(), double_arr_.end()); break;
      case TYPE_STRING:
        msg.string_array.assign(string_arr_.begin(), string_arr_.end()); break;
      default: break;
    }
  }
  return msg;
}

}  // namespace opcua_ros2
