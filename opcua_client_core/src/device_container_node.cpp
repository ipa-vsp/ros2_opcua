#include "opcua_client_core/device_container.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto device_container = std::make_shared<ros2_opcua::DeviceContainer>(executor);
    std::thread contInit([&device_container]() { device_container->init(); });

    executor->add_node(device_container);
    executor->spin();
    rclcpp::shutdown();
    return 0;
}
