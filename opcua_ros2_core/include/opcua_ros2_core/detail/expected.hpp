// Copyright 2024 Vishnuprasad Prachandabhanu
// SPDX-License-Identifier: Apache-2.0
// Thin re-export: use system tl::expected on Ubuntu 24.04 / GCC 13.
#pragma once

#include <tl/expected.hpp>

namespace opcua_ros2
{
using tl::expected;
using tl::unexpected;
using tl::make_unexpected;
}  // namespace opcua_ros2
