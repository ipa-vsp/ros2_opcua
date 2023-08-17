#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "open62541pp/open62541pp.h"


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node_ = rclcpp::Node::make_shared("opcua_example_node", options);
    auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    exec->add_node(node_);

    opcua::Client client;
    client.connect("opc.tcp://192.168.250.1:4840");

    opcua::Node node = client.getNode(opcua::VariableId::Server_ServerStatus_CurrentTime);
    const auto dt = node.readScalar<opcua::DateTime>();

    std::cout << "Server date (UTC): " << dt.format("%Y-%m-%d %H:%M:%S") << std::endl;

    opcua::Variant variant;
    client.getNode({4, "MY_RAND2"}).readValue(variant);

    const auto* value = static_cast<double*>(variant.getScalar());
    RCLCPP_INFO(node_->get_logger(), "Rand data: %f", value);



    // while(rclcpp::ok())
    // {
    //     auto rand = client.getNode(opcua::VariableId::new_Controller_0_GlobalVar_MY_RAND2);
    //     const auto rr = rand.readScalar<double>();
    //     RCLCPP_INFO(node_->get_logger(), "Rand data: %f", rr);
    //     rclcpp::sleep_for(std::chrono::seconds(1));
    //     exec->spin_once();
    // }

    client.disconnect();
    rclcpp::shutdown();
    return 0;
}