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
#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define OPCUA_ROS2_CORE_EXPORT __attribute__((dllexport))
    #define OPCUA_ROS2_CORE_IMPORT __attribute__((dllimport))
  #else
    #define OPCUA_ROS2_CORE_EXPORT __declspec(dllexport)
    #define OPCUA_ROS2_CORE_IMPORT __declspec(dllimport)
  #endif
  #ifdef OPCUA_ROS2_CORE_BUILDING_DLL
    #define OPCUA_ROS2_CORE_PUBLIC OPCUA_ROS2_CORE_EXPORT
  #else
    #define OPCUA_ROS2_CORE_PUBLIC OPCUA_ROS2_CORE_IMPORT
  #endif
  #define OPCUA_ROS2_CORE_PUBLIC_TYPE OPCUA_ROS2_CORE_PUBLIC
  #define OPCUA_ROS2_CORE_LOCAL
#else
  #define OPCUA_ROS2_CORE_EXPORT __attribute__((visibility("default")))
  #define OPCUA_ROS2_CORE_IMPORT
  #if __GNUC__ >= 4
    #define OPCUA_ROS2_CORE_PUBLIC __attribute__((visibility("default")))
    #define OPCUA_ROS2_CORE_LOCAL  __attribute__((visibility("hidden")))
  #else
    #define OPCUA_ROS2_CORE_PUBLIC
    #define OPCUA_ROS2_CORE_LOCAL
  #endif
  #define OPCUA_ROS2_CORE_PUBLIC_TYPE
#endif
