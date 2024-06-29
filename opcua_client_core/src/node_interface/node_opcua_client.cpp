#include "opcua_client_core/node_interface/node_opcua_client.hpp"
#include <memory> // Add this line
#include <rclcpp_lifecycle/lifecycle_node.hpp>

template <class NODETYPE> bool ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::createClient()
{
    if (opcua_client_ == nullptr)
    {
        opcua_client_ = std::make_unique<opcua::Client>();
    }
    return true;
}

template <class NODETYPE>
void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::connectClient(const std::string &endpointURL)
{
    std::unique_lock<std::shared_mutex> lock(opcua_client_mutex_);
    opcua_client_->connect(endpointURL);
}

template <class NODETYPE> void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::disconnectClient()
{
    std::unique_lock<std::shared_mutex> lock(opcua_client_mutex_);
    opcua_client_->disconnect();
}

template <class NODETYPE> void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::createSubscription()
{
    std::unique_lock<std::shared_mutex> lock(opcua_client_mutex_);
    opcua_client_->createSubscription();
}

template <class NODETYPE> void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::createMonitoredItems()
{
    std::unique_lock<std::shared_mutex> lock(opcua_client_mutex_);
    // opcua_client_->createMonitoredItems();
}

template <class NODETYPE>
opcua::Variant ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::readValue(const opcua::NodeId &nodeId)
{
    std::shared_lock<std::shared_mutex> lock(opcua_rw_mutex_);
    return opcua_client_->getNode(nodeId).readValue();
}

template <class NODETYPE>
void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::writeValue(const opcua::NodeId &nodeId,
                                                                       const opcua::Variant &value)
{
    std::unique_lock<std::shared_mutex> lock(opcua_rw_mutex_);
    opcua_client_->getNode(nodeId).writeValue(value);
}

template <class NODETYPE> void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::browse()
{
    std::shared_lock<std::shared_mutex> lock(opcua_rw_mutex_);
    // opcua_client_->browse();
}

template <class NODETYPE> void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::callMethod()
{
    std::shared_lock<std::shared_mutex> lock(opcua_rw_mutex_);
    // opcua_client_->callMethod();
}

template <class NODETYPE> void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::init()
{
    RCLCPP_INFO(node_->get_logger(), "Initializing OPC UA Client");
    node_->declare_parameter("endpoint_url", "");
    node_->declare_parameter("config", "");
}

template <class NODETYPE> void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::configure()
{
    RCLCPP_INFO(node_->get_logger(), "Configuring OPC UA Client");
    std::string config;
    node_->get_parameter("config", config);
    node_->get_parameter("endpoint_url", endpoint_url_);

    RCLCPP_INFO(node_->get_logger(), "Loaded2 endpoint url: %s", endpoint_url_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Loaded2 config: %s", config.c_str());

    try {
        this->config_ = YAML::LoadFile(config);

        if(this->config_.IsNull()) {
            RCLCPP_ERROR(node_->get_logger(), "No config found");
            return;
        }

        if (!this->config_.IsMap()) {
            RCLCPP_ERROR(node_->get_logger(), "Config is not a map. Please check the YAML structure.");
            return;
        }

        if (this->config_["endpoint_url"]) {
            auto endpoint = this->config_["endpoint_url"].as<std::string>();
            RCLCPP_INFO(node_->get_logger(), "Loaded endpoint: %s", endpoint.c_str());
        } else {
            RCLCPP_ERROR(node_->get_logger(), "The 'endpoint_url' key is missing in the YAML configuration.");
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception parsing YAML config: %s", e.what());
    }
}

template <class NODETYPE> void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::activate()
{
    RCLCPP_INFO(node_->get_logger(), "Activating OPC UA Client");
    // connectClient(endpoint_url_);
    // createSubscription();
    // createMonitoredItems();

    YAML::Node variables = this->config_["variables"];
    if (variables.IsNull())
    {
        RCLCPP_ERROR(node_->get_logger(), "No variables found in config");
        return;
    }
    for (YAML::const_iterator it = variables.begin(); it != variables.end(); ++it)
    {
        variableInfo variable_info;
        variable_info.name = it->first.as<std::string>();
        YAML::Node variable = it->second;

        variable_info.index = variable["index"].as<uint8_t>();
        variable_info.type = variable["type"].as<std::string>();
        variable_info.typeID = variable["typeID"].as<std::string>();
        variable_info.bynaryTypeID = variable["bynaryTypeID"].as<std::string>();

        std::string description = variable["description"].as<std::string>();

        RCLCPP_INFO(node_->get_logger(), "Activating variable '%s', description '%s' ", variable_info.name.c_str(), description.c_str());
        RCLCPP_INFO(node_->get_logger(), "Index: %d, Type: %s, TypeID: %s, BynaryTypeID: %s", variable_info.index, variable_info.type.c_str(), variable_info.typeID.c_str(), variable_info.bynaryTypeID.c_str());

        if(variable_info.type == "struct")
        {
            RCLCPP_INFO(node_->get_logger(), "Struct type");
            YAML::Node elements = variable["elements"];
            if(elements.IsNull())
            {
                RCLCPP_ERROR(node_->get_logger(), "No elements found in struct");
                return;
            }
            for (YAML::const_iterator it = elements.begin(); it != elements.end(); ++it)
            {
                RCLCPP_INFO(node_->get_logger(), "Element: %s, Type: %s", it->first.as<std::string>().c_str(), it->second.as<std::string>().c_str());
                std::map<std::string, std::string> element;
                element["name"] = it->first.as<std::string>();
                element["type"] = it->second.as<std::string>();
                variable_info.elements.push_back(element);
                RCLCPP_INFO(node_->get_logger(), "Element: %s, Type: %s", element["name"].c_str(), element["type"].c_str());
            }
        }
        variables_.push_back(variable_info);
    }
}

template <class NODETYPE> void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::deactivate()
{
    RCLCPP_INFO(node_->get_logger(), "Deactivating OPC UA Client");
    // disconnectClient();
}

template <class NODETYPE> void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::cleanup()
{
    RCLCPP_INFO(node_->get_logger(), "Cleaning up OPC UA Client");
    //     if(this->getClient().isConnected())
    //     {
    //         this->getClient().disconnect();
    //     }
    //     opcua_client_.reset();
}

template <class NODETYPE> void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::shutdown()
{
    RCLCPP_INFO(node_->get_logger(), "Shutting down OPC UA Client");
    // if(this->getClient().isConnected())
    // {
    //     this->getClient().disconnect();
    // }
}

template class ros2_opcua::node_interface::NodeOpcUAClient<rclcpp_lifecycle::LifecycleNode>;
