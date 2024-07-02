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

    try
    {
        this->config_ = YAML::LoadFile(config);

        if (this->config_.IsNull())
        {
            RCLCPP_ERROR(node_->get_logger(), "No config found");
            return;
        }

        if (!this->config_.IsMap())
        {
            RCLCPP_ERROR(node_->get_logger(), "Config is not a map. Please check the YAML structure.");
            return;
        }

        if (this->config_["endpoint_url"])
        {
            auto endpoint = this->config_["endpoint_url"].as<std::string>();
            RCLCPP_INFO(node_->get_logger(), "Loaded endpoint: %s", endpoint.c_str());
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "The 'endpoint_url' key is missing in the YAML configuration.");
        }
    }
    catch (const YAML::Exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Exception parsing YAML config: %s", e.what());
    }
}

template <class NODETYPE> void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::activate()
{
    RCLCPP_INFO(node_->get_logger(), "Activating OPC UA Client");
    // connectClient(endpoint_url_);
    // createSubscription();
    // createMonitoredItems();
    loadVariables(config_);
    updateVariables();
    // todo: update the variables in the OPC UA client
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

template <class NODETYPE>
void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::loadVariables(const YAML::Node &config)
{
    YAML::Node variables = config["variables"];

    if (variables.IsNull())
    {
        RCLCPP_ERROR(node_->get_logger(), "No variables found in config");
        return;
    }

    if (!variables.IsMap())
    {
        RCLCPP_ERROR(node_->get_logger(), "Variables is not a map. Please check the YAML structure.");
        return;
    }

    for (YAML::const_iterator it = variables.begin(); it != variables.end(); ++it)
    {
        variableInfo variable_info;
        variable_info.name = it->first.as<std::string>();
        YAML::Node variable = it->second;

        try
        {
            variable_info.index = variable["index"].as<uint8_t>();
            variable_info.type = variable["type"].as<std::string>();
            variable_info.typeID = variable["typeID"].as<std::string>();
            variable_info.bynaryTypeID = variable["bynaryTypeID"].as<std::string>();
            std::string description = variable["description"].as<std::string>();

            if (variable_info.type == "struct")
            {
                try
                {
                    YAML::Node elements = variable["elements"];
                    if (elements.IsNull())
                    {
                        RCLCPP_ERROR(node_->get_logger(), "[on_activate] No elements found in struct");
                        return;
                    }
                    if (!elements.IsSequence())
                    { // Changed to IsSequence to match YAML structure
                        RCLCPP_ERROR(node_->get_logger(),
                                     "[on_activate] Elements is not a sequence. Please check the YAML structure.");
                        return;
                    }

                    for (YAML::const_iterator elem_it = elements.begin(); elem_it != elements.end(); ++elem_it)
                    {
                        auto element_map = *elem_it;
                        std::string element_name = element_map["name"].as<std::string>();
                        std::string element_type = element_map["type"].as<std::string>();

                        std::map<std::string, std::string> element;
                        element["name"] = element_name;
                        element["type"] = element_type;
                        variable_info.elements.push_back(element);
                    }
                }
                catch (const YAML::Exception &e)
                {
                    RCLCPP_ERROR(node_->get_logger(), "[on_activate] Exception parsing YAML elements: %s", e.what());
                }
                RCLCPP_INFO(node_->get_logger(), "[on_activate] Loaded Variable: %s with elements",
                            variable_info.name.c_str());
            }
            else
            {
                std::map<std::string, std::string> element;
                element["name"] = "";
                element["type"] = "";
                variable_info.elements.push_back(element);
                RCLCPP_INFO(node_->get_logger(), "[on_activate] Loaded Variable: %s, without elements",
                            variable_info.name.c_str());
            }
            variables_.push_back(variable_info);
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "[on_activate] Exception parsing YAML variable: %s", e.what());
        }
    }

    RCLCPP_INFO(node_->get_logger(), "[on_activate] Finally Loaded %d variables", (int)variables_.size());
}

template <class NODETYPE> void ros2_opcua::node_interface::NodeOpcUAClient<NODETYPE>::updateVariables()
{
    RCLCPP_INFO(node_->get_logger(), "Updating variables");
    registerTypes();
    RCLCPP_INFO(node_->get_logger(), "Registered types");
    for (auto variable : variables_)
    {
        if (variable.type == "struct")
        {
            // auto out = opcua_client_->getNode({variable.index, variable.name}).readValue();
            using Type = decltype(struct_type_registry.getType<void>(variable.name));
            RCLCPP_INFO(node_->get_logger(), "Variable %s has type %s", variable.name.c_str(),
                        struct_type_registry.getType<Type>(variable.name).name());
            // auto data = out.getScalarCopy<Type>();
        }
    }
}

template class ros2_opcua::node_interface::NodeOpcUAClient<rclcpp_lifecycle::LifecycleNode>;
