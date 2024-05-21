#ifndef NODE_OPCUA_CLIENT_HPP__
#define NODE_OPCUA_CLIENT_HPP__

#include "opcua_client_core/node_interface/node_opcua_client_interface.hpp"

namespace ros2_opcua
{
    namespace node_interface
    {
        class NodeOpcUAClient : public NodeOpcUAClientInterface
        {
        public:
            NodeOpcUAClient() = default;
            ~NodeOpcUAClient() = default;

            void configure() override;
            void activate() override;
            void deactivate() override;
            void cleanup() override;
            void shutdown() override;

        private:
            std::unique_ptr<opcua::Client> client_{nullptr};
            mutable std::shared_mutex client_mutex_{};
            mutable std::shared_mutex rw_mutex_{};
        };
    }
}

#endif //NODE_OPCUA_CLIENT_HPP__