// Copyright 2024 Vishnuprasad Prachandabhanu
// SPDX-License-Identifier: Apache-2.0
// Internal header — allowed to include open62541pp headers.
// Never include from public (non-detail) headers.
#pragma once

#include <cstring>
#include <open62541pp/types.hpp>
#include <open62541pp/detail/open62541/common.h>  // UA_TYPES array

#include "opcua_ros2_core/node_id.hpp"
#include "opcua_ros2_core/status_code.hpp"
#include "opcua_ros2_core/types.hpp"
#include "opcua_ros2_core/variant.hpp"

namespace opcua_ros2::detail
{

// --- NodeId ---

inline opcua::NodeId to_ua(const NodeId & id)
{
  switch (id.identifier_type()) {
    case NodeId::IdentifierType::Numeric:
      return opcua::NodeId{id.namespace_index(), id.numeric_id()};
    case NodeId::IdentifierType::String:
      return opcua::NodeId{id.namespace_index(), opcua::String{id.string_id()}};
    case NodeId::IdentifierType::Guid: {
        const auto & g = id.guid_id();
        UA_Guid ua_g{};
        ua_g.data1 = (uint32_t(g[0]) << 24) | (uint32_t(g[1]) << 16) | (uint32_t(g[2]) << 8) | g[3];
        ua_g.data2 = (uint16_t(g[4]) << 8) | g[5];
        ua_g.data3 = (uint16_t(g[6]) << 8) | g[7];
        for (int i = 0; i < 8; ++i) {
          ua_g.data4[i] = g[8 + i];
        }                                                  // NOLINT
        return opcua::NodeId{id.namespace_index(), opcua::Guid{ua_g}};
      }
    case NodeId::IdentifierType::Opaque: {
        const auto & op = id.opaque_id();
        opcua::ByteString bs{opcua::Span<const uint8_t>{op.data(), op.size()}};
        return opcua::NodeId{id.namespace_index(), std::move(bs)};
      }
  }
  return {};
}

inline NodeId to_core(const opcua::NodeId & id)
{
  switch (id.identifierType()) {
    case opcua::NodeIdType::Numeric:
      return NodeId{id.namespaceIndex(), id.identifier<uint32_t>()};
    case opcua::NodeIdType::String: {
        const auto & s = id.identifier<opcua::String>();
        return NodeId{id.namespaceIndex(),
          std::string{reinterpret_cast<const char *>(s.data()), s.size()}};
      }
    case opcua::NodeIdType::Guid: {
        const UA_Guid & ua_g = *static_cast<const UA_Guid *>(
          id.identifier<opcua::Guid>().handle());
        Guid g{};
        g[0] = (ua_g.data1 >> 24) & 0xFF; g[1] = (ua_g.data1 >> 16) & 0xFF;
        g[2] = (ua_g.data1 >> 8) & 0xFF;  g[3] = ua_g.data1 & 0xFF;
        g[4] = (ua_g.data2 >> 8) & 0xFF;  g[5] = ua_g.data2 & 0xFF;
        g[6] = (ua_g.data3 >> 8) & 0xFF;  g[7] = ua_g.data3 & 0xFF;
        for (int i = 0; i < 8; ++i) {
          g[8 + i] = ua_g.data4[i];
        }                                                  // NOLINT
        return NodeId{id.namespaceIndex(), g};
      }
    case opcua::NodeIdType::ByteString: {
        const auto & bs = id.identifier<opcua::ByteString>();
        return NodeId{id.namespaceIndex(),
          std::vector<uint8_t>(bs.begin(), bs.end())};
      }
  }
  return {};
}

// --- Variant ---
inline Variant to_core_variant(const opcua::Variant & v)
{
  if (v.empty()) {return {}}
  if (v.isScalar()) {
    if (v.isType<UA_Boolean>()) {return Variant::make_bool(*static_cast<const bool *>(v.data()));}
    if (v.isType<UA_SByte>()) {return Variant::make_int8(*static_cast<const int8_t *>(v.data()));}
    if (v.isType<UA_Byte>()) {return Variant::make_uint8(*static_cast<const uint8_t *>(v.data()));}
    if (v.isType<UA_Int16>()) {return Variant::make_int16(*static_cast<const int16_t *>(v.data()));}
    if (v.isType<UA_UInt16>()) {
      return Variant::make_uint16(*static_cast<const uint16_t *>(v.data()));
    }
    if (v.isType<UA_Int32>()) {return Variant::make_int32(*static_cast<const int32_t *>(v.data()));}
    if (v.isType<UA_UInt32>()) {
      return Variant::make_uint32(*static_cast<const uint32_t *>(v.data()));
    }
    if (v.isType<UA_Int64>()) {return Variant::make_int64(*static_cast<const int64_t *>(v.data()));}
    if (v.isType<UA_UInt64>()) {
      return Variant::make_uint64(*static_cast<const uint64_t *>(v.data()));
    }
    if (v.isType<UA_Float>()) {return Variant::make_float(*static_cast<const float *>(v.data()));}
    if (v.isType<UA_Double>()) {
      return Variant::make_double(*static_cast<const double *>(v.data()));
    }
    // Use UA_DataType* overload to avoid TypeRegistry<UA_String> ambiguity
    if (v.isType(&UA_TYPES[UA_TYPES_STRING])) {
      const auto * s = static_cast<const UA_String *>(v.data());
      if (s && s->data) {
        return Variant::make_string(std::string{reinterpret_cast<const char *>(s->data),
                   s->length});
      }
    }
    if (v.isType<UA_StatusCode>()) {
      StatusCode sc;
      sc.value = *static_cast<const uint32_t *>(v.data());
      sc.name = std::string{UA_StatusCode_name(sc.value)};
      return Variant::make_status_code(sc);
    }
  }
  if (!v.isScalar()) {  // array
    if (v.isType<UA_Double>()) {
      const double * ptr = static_cast<const double *>(v.data());
      return Variant::make_double_array(
        std::vector<double>(ptr, ptr + v.arrayLength()));
    }
    if (v.isType<UA_Int64>()) {
      const int64_t * ptr = static_cast<const int64_t *>(v.data());
      return Variant::make_int_array(
        std::vector<int64_t>(ptr, ptr + v.arrayLength()));
    }
    if (v.isType<UA_UInt64>()) {
      const uint64_t * ptr = static_cast<const uint64_t *>(v.data());
      return Variant::make_uint_array(
        std::vector<uint64_t>(ptr, ptr + v.arrayLength()));
    }
  }
  return {};
}

inline opcua::Variant to_ua_variant(const Variant & v)
{
  if (v.is_null()) {return {}}
  if (v.is_scalar()) {
    switch (*v.type_tag()) {
      case Variant::TYPE_BOOL:   {UA_Boolean b = v.get_bool(); opcua::Variant r; r.assign(b);
          return r;}
      case Variant::TYPE_INT8:   {auto n = static_cast<int8_t>(v.get_int());   opcua::Variant r;
          r.assign(n); return r;}
      case Variant::TYPE_INT16:  {auto n = static_cast<int16_t>(v.get_int());  opcua::Variant r;
          r.assign(n); return r;}
      case Variant::TYPE_INT32:  {auto n = static_cast<int32_t>(v.get_int());  opcua::Variant r;
          r.assign(n); return r;}
      case Variant::TYPE_INT64:  {int64_t n = v.get_int();   opcua::Variant r; r.assign(n);
          return r;}
      case Variant::TYPE_UINT8:  {auto n = static_cast<uint8_t>(v.get_uint());  opcua::Variant r;
          r.assign(n); return r;}
      case Variant::TYPE_UINT16: {auto n = static_cast<uint16_t>(v.get_uint()); opcua::Variant r;
          r.assign(n); return r;}
      case Variant::TYPE_UINT32: {auto n = static_cast<uint32_t>(v.get_uint()); opcua::Variant r;
          r.assign(n); return r;}
      case Variant::TYPE_UINT64: {uint64_t n = v.get_uint(); opcua::Variant r; r.assign(n);
          return r;}
      case Variant::TYPE_FLOAT:  {auto f = static_cast<float>(v.get_double()); opcua::Variant r;
          r.assign(f); return r;}
      case Variant::TYPE_DOUBLE: {double d = v.get_double(); opcua::Variant r; r.assign(d);
          return r;}
      // Note: String intentionally excluded here — UA_String is ambiguous in TypeRegistry.
      // String support can be added later using explicit UA_DataType overload.
      default: break;
    }
  }
  if (v.is_array()) {
    switch (*v.type_tag()) {
      case Variant::TYPE_DOUBLE: {
          auto sp = v.get_double_array();
          std::vector<double> vec(sp.begin(), sp.end());
          opcua::Variant r;
          r.assign(vec);
          return r;
        }
      case Variant::TYPE_INT64: {
          auto sp = v.get_int_array();
          std::vector<int64_t> vec(sp.begin(), sp.end());
          opcua::Variant r;
          r.assign(vec);
          return r;
        }
      default: break;
    }
  }
  return {};
}

}  // namespace opcua_ros2::detail
