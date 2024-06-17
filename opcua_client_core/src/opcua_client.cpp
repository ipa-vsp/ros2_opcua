#include "opcua_client_core/opcua_client.hpp"

void ros2_opcua::LifecycleOpcUAClient::init()
{
    node_opcua_client_->init();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ros2_opcua::LifecycleOpcUAClient::on_configure(const rclcpp_lifecycle::State & state)
{
    node_opcua_client_->configure();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ros2_opcua::LifecycleOpcUAClient::on_activate(const rclcpp_lifecycle::State & state)
{
    node_opcua_client_->activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ros2_opcua::LifecycleOpcUAClient::on_deactivate(const rclcpp_lifecycle::State & state)
{
    node_opcua_client_->deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ros2_opcua::LifecycleOpcUAClient::on_cleanup(const rclcpp_lifecycle::State & state)
{
    node_opcua_client_->cleanup();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ros2_opcua::LifecycleOpcUAClient::on_shutdown(const rclcpp_lifecycle::State & state)
{
    node_opcua_client_->shutdown();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_opcua::LifecycleOpcUAClient)