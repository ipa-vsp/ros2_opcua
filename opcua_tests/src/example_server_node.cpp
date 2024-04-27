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
#include <iostream>

#include "opcua_tests/custom_datatype.h"
#include "open62541pp/open62541pp.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node_ = rclcpp::Node::make_shared("opcua_example_server_node", options);
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  opcua::Server server;

  // Get custom type definitions from common header
  const auto& dataTypePoint = getPointDataType();
  const auto& dataTypeMeasurements = getMeasurementsDataType();
  const auto& dataTypeOpt = getOptDataType();
  const auto& dataTypeUni = getUniDataType();
  const auto& dataTypeColor = getColorDataType();

  // Provide custom data type definitions to server
  server.setCustomDataTypes({
      dataTypePoint,
      dataTypeMeasurements,
      dataTypeOpt,
      dataTypeUni,
      dataTypeColor,
  });

  // Add data type nodes
  auto nodeStructureDataType = server.getNode(opcua::DataTypeId::Structure);
  nodeStructureDataType.addDataType(dataTypePoint.getTypeId(), "Point");
  nodeStructureDataType.addDataType(dataTypeMeasurements.getTypeId(), "Measurements");
  nodeStructureDataType.addDataType(dataTypeOpt.getTypeId(), "Opt");
  nodeStructureDataType.addDataType(dataTypeUni.getTypeId(), "Uni");
  auto nodeEnumerationDataType = server.getNode(opcua::DataTypeId::Enumeration);
  nodeEnumerationDataType.addDataType(dataTypeColor.getTypeId(), "Color")
      .addProperty({ 0, 0 },  // auto-generate node id
                   "EnumValues",
                   opcua::VariableAttributes{}
                       .setDataType<opcua::EnumValueType>()
                       .setValueRank(opcua::ValueRank::OneDimension)
                       .setArrayDimensions({ 0 })
                       .setValueArray(opcua::Span<const opcua::EnumValueType>{
                           { 0, { "", "Red" }, {} },
                           { 1, { "", "Green" }, {} },
                           { 2, { "", "Yellow" }, {} },
                       }))
      .addModellingRule(opcua::ModellingRule::Mandatory);

  // Add variable type nodes (optional)
  auto nodeBaseDataVariableType = server.getNode(opcua::VariableTypeId::BaseDataVariableType);
  auto nodeVariableTypePoint =
      nodeBaseDataVariableType.addVariableType({ 1, 4243 }, "PointType",
                                               opcua::VariableTypeAttributes{}
                                                   .setDataType(dataTypePoint.getTypeId())
                                                   .setValueRank(opcua::ValueRank::ScalarOrOneDimension)
                                                   .setValueScalar(Point{ 1, 2, 3 }, dataTypePoint));
  auto nodeVariableTypeMeasurement =
      nodeBaseDataVariableType.addVariableType({ 1, 4444 }, "MeasurementsType",
                                               opcua::VariableTypeAttributes{}
                                                   .setDataType(dataTypeMeasurements.getTypeId())
                                                   .setValueRank(opcua::ValueRank::Scalar)
                                                   .setValueScalar(Measurements{}, dataTypeMeasurements));
  auto nodeVariableTypeOpt = nodeBaseDataVariableType.addVariableType({ 1, 4645 }, "OptType",
                                                                      opcua::VariableTypeAttributes{}
                                                                          .setDataType(dataTypeOpt.getTypeId())
                                                                          .setValueRank(opcua::ValueRank::Scalar)
                                                                          .setValueScalar(Opt{}, dataTypeOpt));
  auto nodeVariableTypeUni = nodeBaseDataVariableType.addVariableType({ 1, 4846 }, "UniType",
                                                                      opcua::VariableTypeAttributes{}
                                                                          .setDataType(dataTypeUni.getTypeId())
                                                                          .setValueRank(opcua::ValueRank::Scalar)
                                                                          .setValueScalar(Uni{}, dataTypeUni));

  // Add variable nodes with some values
  auto nodeObjects = server.getObjectsNode();

  const Point point{ 3.0, 4.0, 5.0 };
  nodeObjects.addVariable({ 1, "Point" }, "Point",
                          opcua::VariableAttributes{}
                              .setDataType(dataTypePoint.getTypeId())
                              .setValueRank(opcua::ValueRank::Scalar)
                              .setValueScalar(point, dataTypePoint),
                          nodeVariableTypePoint.getNodeId());

  const std::vector<Point> pointVec{ { 1.0, 2.0, 3.0 }, { 4.0, 5.0, 6.0 } };
  nodeObjects.addVariable({ 1, "PointVec" }, "PointVec",
                          opcua::VariableAttributes{}
                              .setDataType(dataTypePoint.getTypeId())
                              .setArrayDimensions({ 0 })  // single dimension but unknown in size
                              .setValueRank(opcua::ValueRank::OneDimension)
                              .setValueArray(pointVec, dataTypePoint),
                          nodeVariableTypePoint.getNodeId());

  std::vector<float> measurementsValues{ 19.1F, 20.2F, 19.7F };
  const Measurements measurements{
    opcua::String("Test description"),
    measurementsValues.size(),
    measurementsValues.data(),
  };
  nodeObjects.addVariable({ 1, "Measurements" }, "Measurements",
                          opcua::VariableAttributes{}
                              .setDataType(dataTypeMeasurements.getTypeId())
                              .setValueRank(opcua::ValueRank::Scalar)
                              .setValueScalar(measurements, dataTypeMeasurements),
                          nodeVariableTypeMeasurement.getNodeId());

  float optC = 10.10F;
  const Opt opt{ 3, nullptr, &optC };
  nodeObjects.addVariable({ 1, "Opt" }, "Opt",
                          opcua::VariableAttributes{}
                              .setDataType(dataTypeOpt.getTypeId())
                              .setValueRank(opcua::ValueRank::Scalar)
                              .setValueScalar(opt, dataTypeOpt),
                          nodeVariableTypeOpt.getNodeId());

  Uni uni{};
  uni.switchField = UniSwitch::OptionB;
  uni.fields.optionB = UA_STRING_STATIC("test string");  // NOLINT
  nodeObjects.addVariable({ 1, "Uni" }, "Uni",
                          opcua::VariableAttributes{}
                              .setDataType(dataTypeUni.getTypeId())
                              .setValueRank(opcua::ValueRank::Scalar)
                              .setValueScalar(uni, dataTypeUni),
                          nodeVariableTypeUni.getNodeId());

  nodeObjects.addVariable({ 1, "Color" }, "Color",
                          opcua::VariableAttributes{}
                              .setDataType(dataTypeColor.getTypeId())
                              .setValueRank(opcua::ValueRank::Scalar)
                              .setValueScalar(Color::Green, dataTypeColor));

  server.run();

  rclcpp::shutdown();
  return 0;
}
