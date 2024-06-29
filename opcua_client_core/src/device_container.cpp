#include "opcua_client_core/device_container.hpp"

void ros2_opcua::DeviceContainer::init()
{
    configure();

    if (!this->load_driver())
    {
        throw std::runtime_error("Failed to load driver");
    }
}

void ros2_opcua::DeviceContainer::configure()
{
    RCLCPP_INFO(this->get_logger(), "Configuring device containers");
    if (!this->get_parameter("package", package_))
    {
        throw std::runtime_error("Failed to get package name");
    }

    if (!this->get_parameter("driver", driver_))
    {
        throw std::runtime_error("Failed to get driver name");
    }

    if (!this->get_parameter("dt_config", dt_config_))
    {
        throw std::runtime_error("Failed to get device type config");
    }

    if (!this->get_parameter("endpoint_url", endpoint_url_))
    {
        throw std::runtime_error("Failed to get endpoint url");
    }

    RCLCPP_INFO(this->get_logger(), "Starting device containers");
    RCLCPP_INFO(this->get_logger(), "Loaded endpoint url: %s", endpoint_url_.c_str());
    RCLCPP_INFO(this->get_logger(), "Loaded package: %s", package_.c_str());
    RCLCPP_INFO(this->get_logger(), "Loaded driver: %s", driver_.c_str());
    RCLCPP_INFO(this->get_logger(), "Loaded data types config: %s", dt_config_.c_str());
}

bool ros2_opcua::DeviceContainer::load_driver()
{
    std::vector<rclcpp::Parameter> params;
    params.push_back(rclcpp::Parameter("config", dt_config_));
    params.push_back(rclcpp::Parameter("endpoint_url", endpoint_url_));
    if (!this->load_components(package_, driver_, params))
    {
        throw std::runtime_error("Failed to load components");
        return false;
    }

    this->add_node_to_executor(this->opcua_client_->get_node_base_interface());
    this->opcua_client_->init();
    return true;
}

bool ros2_opcua::DeviceContainer::load_components(std::string &package_name, std::string &driver_name,
                                                  std::vector<rclcpp::Parameter> &params)
{
    RCLCPP_INFO(this->get_logger(), "Loading components from package: %s", package_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Loading driver: %s", driver_name.c_str());

    std::string node_name = "opcua_client_node";
    std::string resource_index("rclcpp_components");
    std::vector<ComponentResource> components = this->get_component_resources(package_name, resource_index);

    for (auto it = components.begin(); it != components.end(); it++)
    {
        if (it->first.compare(driver_name) == 0)
        {
            std::shared_ptr<rclcpp_components::NodeFactory> factory = this->create_component_factory(*it);
            rclcpp::NodeOptions options;
            options.use_global_arguments(false);
            options.use_intra_process_comms(true);
            std::vector<std::string> remap_rules;
            remap_rules.push_back("--ros-args");
            remap_rules.push_back("-r");
            remap_rules.push_back("__node:=" + node_name);
            options.arguments(remap_rules);
            options.parameter_overrides(params);

            try
            {
                auto wrapper = factory->create_node_instance(options);
                this->opcua_client_ =
                    std::static_pointer_cast<ros2_opcua::OpcUAClientInterface>(wrapper.get_node_instance());
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
                return false;
            }
            return true;
        }
    }
    return false;
}
