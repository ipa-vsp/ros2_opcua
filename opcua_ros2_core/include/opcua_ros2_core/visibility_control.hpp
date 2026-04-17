// Copyright 2024 Vishnuprasad Prachandabhanu
// SPDX-License-Identifier: Apache-2.0
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
