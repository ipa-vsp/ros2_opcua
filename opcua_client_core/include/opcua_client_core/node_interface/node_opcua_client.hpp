#ifndef NODE_OPCUA_CLIENT_HPP__
#define NODE_OPCUA_CLIENT_HPP__

#include "opcua_client_core/node_interface/node_opcua_client_interface.hpp"

namespace ros2_opcua
{
    namespace node_interface
    {
        template <class NODETYPE>
        class NodeOpcUAClient : public NodeOpcUAClientInterface
        {
        public:
            NodeOpcUAClient(NODETYPE *node)
            {
                node_ = node;
            }

            void createClient();
            void connectClient();
            void disconnectClient();
            void createSubscription();
            void createMonitoredItems();
            void readValue();
            void writeValue();
            void browse();
            void callMethod();

            void configure() override;
            void activate() override;
            void deactivate() override;
            void cleanup() override;
            void shutdown() override;

        private:
            NODETYPE *node_{nullptr};
            std::unique_ptr<opcua::Client> client_{nullptr};
            mutable std::shared_mutex client_mutex_{};
            mutable std::shared_mutex rw_mutex_{};
        };
    }
}

#endif //NODE_OPCUA_CLIENT_HPP__