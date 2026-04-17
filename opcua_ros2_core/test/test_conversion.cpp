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

#include "opcua_ros2_core/conversion.hpp"
#include "opcua_ros2_core/variant.hpp"
#include "opcua_ros2_core/node_id.hpp"
#include "opcua_ros2_core/types.hpp"

using namespace opcua_ros2;
using namespace opcua_ros2::conversion;

// --- NodeId round-trips ---
TEST(Conversion, NodeIdNumericRoundTrip) {
  NodeId orig{3, uint32_t{1234}};
  auto msg = to_ros(orig);
  auto back = to_core(msg);
  EXPECT_EQ(orig, back);
}

TEST(Conversion, NodeIdStringRoundTrip) {
  NodeId orig{2, std::string{"MyNode"}};
  auto msg = to_ros(orig);
  auto back = to_core(msg);
  EXPECT_EQ(orig, back);
}

TEST(Conversion, NodeIdGuidRoundTrip) {
  Guid g = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
  NodeId orig{1, g};
  auto msg = to_ros(orig);
  auto back = to_core(msg);
  EXPECT_EQ(orig, back);
}

// --- StatusCode ---
TEST(Conversion, StatusCodeRoundTrip) {
  StatusCode sc;
  sc.value = 0x80350000U;
  sc.name = "BadNodeIdInvalid";
  auto msg = to_ros(sc);
  auto back = to_core(msg);
  EXPECT_EQ(sc.value, back.value);
  EXPECT_EQ(sc.name, back.name);
}

// --- QualifiedName ---
TEST(Conversion, QualifiedNameRoundTrip) {
  QualifiedName qn{3, "MyBrowseName"};
  auto msg = to_ros(qn);
  auto back = to_core(msg);
  EXPECT_EQ(qn, back);
}

// --- Variant scalar round-trips ---
TEST(Conversion, VariantBoolRoundTrip) {
  auto v = Variant::make_bool(true);
  auto msg = to_ros(v);
  auto back = to_core(msg);
  EXPECT_EQ(back.type_tag().value(), Variant::TYPE_BOOL);
  EXPECT_EQ(back.get_bool(), true);
}

TEST(Conversion, VariantInt32RoundTrip) {
  auto v = Variant::make_int32(-42);
  auto msg = to_ros(v);
  auto back = to_core(msg);
  EXPECT_EQ(back.get_int(), -42);
}

TEST(Conversion, VariantUInt64RoundTrip) {
  auto v = Variant::make_uint64(std::numeric_limits<uint64_t>::max());
  auto msg = to_ros(v);
  auto back = to_core(msg);
  EXPECT_EQ(back.get_uint(), std::numeric_limits<uint64_t>::max());
}

TEST(Conversion, VariantDoubleRoundTrip) {
  auto v = Variant::make_double(3.14159);
  auto msg = to_ros(v);
  auto back = to_core(msg);
  EXPECT_DOUBLE_EQ(back.get_double(), 3.14159);
}

TEST(Conversion, VariantStringRoundTrip) {
  auto v = Variant::make_string("hello OPC UA");
  auto msg = to_ros(v);
  auto back = to_core(msg);
  EXPECT_EQ(back.get_string(), "hello OPC UA");
}

// --- Variant array round-trips ---
TEST(Conversion, VariantDoubleArrayRoundTrip) {
  auto v = Variant::make_double_array({1.0, 2.0, 3.0});
  auto msg = to_ros(v);
  auto back = to_core(msg);
  ASSERT_TRUE(back.is_array());
  auto sp = back.get_double_array();
  ASSERT_EQ(sp.size(), 3u);
  EXPECT_DOUBLE_EQ(sp[2], 3.0);
}

TEST(Conversion, VariantIntArrayRoundTrip) {
  auto v = Variant::make_int_array({-1, 0, 1});
  auto msg = to_ros(v);
  auto back = to_core(msg);
  ASSERT_TRUE(back.is_array());
  auto sp = back.get_int_array();
  EXPECT_EQ(sp[0], -1);
}

// --- DataValue ---
TEST(Conversion, DataValueRoundTrip) {
  DataValue dv;
  dv.value = Variant::make_double(9.81);
  dv.status.value = StatusCode::Good;
  dv.source_timestamp_sec = 100;
  dv.source_timestamp_nanosec = 500'000'000u;

  auto msg = to_ros(dv);
  auto back = to_core(msg);
  EXPECT_DOUBLE_EQ(back.value.get_double(), 9.81);
  EXPECT_EQ(back.source_timestamp_sec, 100);
}

// --- NodeId parse ---
TEST(NodeId, ParseNumeric) {
  auto res = NodeId::parse("i=13");
  ASSERT_TRUE(res.has_value());
  EXPECT_EQ(res->numeric_id(), 13u);
  EXPECT_EQ(res->namespace_index(), 0u);
}

TEST(NodeId, ParseNsNumeric) {
  auto res = NodeId::parse("ns=2;i=100");
  ASSERT_TRUE(res.has_value());
  EXPECT_EQ(res->namespace_index(), 2u);
  EXPECT_EQ(res->numeric_id(), 100u);
}

TEST(NodeId, ParseString) {
  auto res = NodeId::parse("ns=1;s=MyNode");
  ASSERT_TRUE(res.has_value());
  EXPECT_EQ(res->string_id(), "MyNode");
}

TEST(NodeId, ParseBadFormat) {
  auto res = NodeId::parse("x=999");
  EXPECT_FALSE(res.has_value());
}

// --- TypeRegistry ---
#include "opcua_ros2_core/type_registry.hpp"
TEST(TypeRegistry, RegisterAndFind) {
  auto & reg = TypeRegistry::instance();
  reg.clear();

  NodeId id{0, uint32_t{9999}};
  int encode_calls = 0;
  reg.register_codec(
    id, "my_pkg/MyType",
    [&encode_calls](const void *) -> std::vector<std::byte> {++encode_calls; return {};},
    [](std::span<const std::byte>, void *) {});

  EXPECT_TRUE(reg.find_encoder(id).has_value());
  EXPECT_TRUE(reg.find_decoder(id).has_value());

  auto enc = reg.find_encoder(id).value();
  enc(nullptr);
  EXPECT_EQ(encode_calls, 1);

  NodeId other{0, uint32_t{1}};
  EXPECT_FALSE(reg.find_encoder(other).has_value());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
