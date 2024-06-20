#ifndef NODE_OPCUA_CLIENT_HPP__
#define NODE_OPCUA_CLIENT_HPP__

#include <yaml-cpp/yaml.h>
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

            void init();
            void configure();
            void activate();
            void deactivate();
            void cleanup();
            void shutdown();

            opcua::Client &getClient() { return *opcua_client_;}

        protected:
            bool createClient();
            void connectClient(const std::string &endpointURL);
            void disconnectClient();
            void createSubscription();
            void createMonitoredItems();
            opcua::Variant readValue(const opcua::NodeId &nodeId);
            void writeValue(const opcua::NodeId &nodeId, const opcua::Variant &value);
            void browse();
            void callMethod();

        protected:
            NODETYPE *node_;
            std::string endpoint_url_;
            std::unique_ptr<opcua::Client> opcua_client_;
            mutable std::shared_mutex opcua_client_mutex_;
            mutable std::shared_mutex opcua_rw_mutex_;

            YAML::Node config_;
        };
    }
}

#endif //NODE_OPCUA_CLIENT_HPP__