//    Copyright 2024 Vishnuprasad Prachandabhanu
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.
#pragma once

#include <cstdint>

#include "open62541pp/open62541pp.h"

// Example struct
struct Point
{
  float x;
  float y;
  float z;
};

const opcua::DataType& getPointDataType()
{
  static const opcua::DataType dt = opcua::DataTypeBuilder<Point>::createStructure("Point", { 1, 4242 }, { 1, 1 })
                                        .addField<&Point::x>("x")
                                        .addField<&Point::y>("y")
                                        .addField<&Point::z>("z")
                                        .build();
  return dt;
}

// Example struct with an array
struct Measurements
{
  opcua::String description;
  size_t measurementsSize;
  float* measurements;
};

const opcua::DataType& getMeasurementsDataType()
{
  static const opcua::DataType dt =
      opcua::DataTypeBuilder<Measurements>::createStructure("Measurements", { 1, 4443 }, { 1, 2 })
          .addField<&Measurements::description>("description")
          .addField<&Measurements::measurementsSize, &Measurements::measurements>("measurements")
          .build();
  return dt;
}

// Example struct with optional fields
struct Opt
{
  int16_t a;
  float* b;
  float* c;
};

const opcua::DataType& getOptDataType()
{
  static const opcua::DataType dt = opcua::DataTypeBuilder<Opt>::createStructure("Opt", { 1, 4644 }, { 1, 3 })
                                        .addField<&Opt::a>("a")
                                        .addField<&Opt::b>("b")
                                        .addField<&Opt::c>("c")
                                        .build();
  return dt;
}

// Example union
enum class UniSwitch : uint32_t
{
  None = 0,
  optional = 1,
  OptionB = 2
};

struct Uni
{
  UniSwitch switchField;

  union
  {
    double optional;
    UA_String optionB;
  } fields;
};

const opcua::DataType& getUniDataType()
{
  static const opcua::DataType dt = opcua::DataTypeBuilder<Uni>::createUnion("Uni", { 1, 4845 }, { 1, 4 })
                                        .addUnionField<&Uni::fields, double>("optional")
                                        .addUnionField<&Uni::fields, UA_String>("optionB", UA_TYPES[UA_TYPES_STRING])
                                        .build();
  return dt;
}

// Example enumeration
enum class Color : int32_t
{
  Red = 0,
  Green = 1,
  Yellow = 2,
};

const opcua::DataType& getColorDataType()
{
  static const opcua::DataType dt = opcua::DataTypeBuilder<Color>::createEnum("Color", { 1, 4946 }, { 1, 5 }).build();
  return dt;
}
