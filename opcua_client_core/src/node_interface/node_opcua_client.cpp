#include "opcua_client_core/node_interface/node_opcua_client.hpp"
#include <memory>  // Add this line
#include <rclcpp_lifecycle/lifecycle_node.hpp>

template <class NODETYPE>
bool ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::createClient()
{
  if (opcua_client_ == nullptr)
  {
    opcua_client_ = std::make_unique<opcua::Client>();
  }
  return true;
}

template <class NODETYPE>
void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::connectClient(const std::string& endpointURL)
{
  std::unique_lock<std::shared_mutex> lock(opcua_client_mutex_);
  opcua_client_->connect(endpointURL);
}

template <class NODETYPE>
void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::disconnectClient()
{
  std::unique_lock<std::shared_mutex> lock(opcua_client_mutex_);
  opcua_client_->disconnect();
}

template <class NODETYPE>
void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::createSubscription()
{
  std::unique_lock<std::shared_mutex> lock(opcua_client_mutex_);
  opcua_client_->createSubscription();
}

template <class NODETYPE>
void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::createMonitoredItems()
{
  std::unique_lock<std::shared_mutex> lock(opcua_client_mutex_);
  // opcua_client_->createMonitoredItems();
}

template <class NODETYPE>
opcua::Variant ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::readValue(const opcua::NodeId& nodeId)
{
  std::shared_lock<std::shared_mutex> lock(opcua_rw_mutex_);
  return opcua_client_->getNode(nodeId).readValue();
}

template <class NODETYPE>
void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::writeValue(const opcua::NodeId& nodeId,
                                                                       const opcua::Variant& value)
{
  std::unique_lock<std::shared_mutex> lock(opcua_rw_mutex_);
  opcua_client_->getNode(nodeId).writeValue(value);
}

template <class NODETYPE>
void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::browse()
{
  std::shared_lock<std::shared_mutex> lock(opcua_rw_mutex_);
  // opcua_client_->browse();
}

template <class NODETYPE>
void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::callMethod()
{
  std::shared_lock<std::shared_mutex> lock(opcua_rw_mutex_);
  // opcua_client_->callMethod();
}

template <class NODETYPE>
void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::init()
{
  RCLCPP_INFO(node_->get_logger(), "Initializing OPC UA Client");
  node_->declare_parameter("endpoint_url", "");
  node_->declare_parameter("config", "");
}

template <class NODETYPE>
void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::configure()
{
  RCLCPP_INFO(node_->get_logger(), "Configuring OPC UA Client");
  // std::config config;
  // node_->get_parameter("endpoint_url", endpoint_url_);
  // node_->get_parameter("config", config);
  // this->config_ = YAML::Load(config);

  // if(this->getClient().isConnected())
  // {
  //     this->getClient().disconnect();
  // }
  // createClient();
}

template <class NODETYPE>
void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activating OPC UA Client");
  // connectClient(endpoint_url_);
  // createSubscription();
  // createMonitoredItems();
}

template <class NODETYPE>
void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating OPC UA Client");
  // disconnectClient();
}

template <class NODETYPE>
void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Cleaning up OPC UA Client");
  //     if(this->getClient().isConnected())
  //     {
  //         this->getClient().disconnect();
  //     }
  //     opcua_client_.reset();
}

template <class NODETYPE>
void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::shutdown()
{
  RCLCPP_INFO(node_->get_logger(), "Shutting down OPC UA Client");
  // if(this->getClient().isConnected())
  // {
  //     this->getClient().disconnect();
  // }
}

template class ros2_opcua::node_interface::NodeOpcUAClient<rclcpp_lifecycle::LifecycleNode>;
