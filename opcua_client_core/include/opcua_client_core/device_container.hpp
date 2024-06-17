#ifndef DEVICE_CONTAINER_HPP__
#define DEVICE_CONTAINER_HPP__

#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp_components/component_manager.hpp>

#include "opcua_client_core/opcua_client.hpp"

namespace ros2_opcua
{
    class DeviceContainer : public rclcpp_components::ComponentManager
    {
        public:
        DeviceContainer(std::weak_ptr<rclcpp::Executor> executor = std::weak_ptr<rclcpp::executors::MultiThreadedExecutor>(),
            std::string node_name = "device_container", const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : rclcpp_components::ComponentManager(executor, node_name, options)
        {
            executor_ = executor;
            cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            this->declare_parameter("package", "");
            this->declare_parameter("driver", "");
        }

        void init();

        private:
            std::weak_ptr<rclcpp::Executor> executor_;
            rclcpp::CallbackGroup::SharedPtr cb_group_;

            std::shared_ptr<ros2_opcua::OpcUAClientInterface> opcua_client_;

            std::string package_;
            std::string driver_;

            void configure();
            bool load_driver();
            bool load_components(std::string &package_name, std::string &driver_name, std::vector<rclcpp::Parameter> &params);

            void add_node_to_executor(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_interface)
            {
                if (auto exec = executor_.lock())
                {
                    RCLCPP_INFO(this->get_logger(), "Added %s to executor", node_interface->get_fully_qualified_name());
                    exec->add_node(node_interface, true);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Executor is not available");
                }
            }
    };
}

#endif //DEVICE_CONTAINER_HPP__