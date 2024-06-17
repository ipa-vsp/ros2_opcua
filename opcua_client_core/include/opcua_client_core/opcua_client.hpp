#ifndef OPACUA_CLIENT_HPP_
#define OPACUA_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "opcua_client_core/node_interface/node_opcua_client.hpp"

namespace ros2_opcua
{
    class OpcUAClientInterface
    {
    public:
        OpcUAClientInterface() = default;
        virtual ~OpcUAClientInterface() = default;

        virtual void init() = 0;
        virtual void configure() = 0;
        virtual void activate() = 0;
        virtual void deactivate() = 0;
        virtual void cleanup() = 0;
        virtual void shutdown() = 0;

        virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() = 0;
    };

    class LifecycleOpcUAClient : public OpcUAClientInterface, public rclcpp_lifecycle::LifecycleNode
    {
        protected:
            std::shared_ptr<ros2_opcua::node_interface::NodeOpcUAClientInterface> node_opcua_client_;
        public:
            explicit LifecycleOpcUAClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()): 
                rclcpp_lifecycle::LifecycleNode("opcua_client", options)
            {
                node_opcua_client_ = std::make_shared<ros2_opcua::node_interface::NodeOpcUAClient<rclcpp_lifecycle::LifecycleNode>>(this);
            }

            virtual void init() override;

            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);

            rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() override
            {
                return rclcpp_lifecycle::LifecycleNode::get_node_base_interface();
            }
    };
}

#endif // OPACUA_CLIENT_HPP_