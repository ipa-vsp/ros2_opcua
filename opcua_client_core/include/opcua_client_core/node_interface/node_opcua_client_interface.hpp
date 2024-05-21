#ifndef NODE_OPCUA_CLIENT_INTERFACE_HPP__
#define NODE_OPCUA_CLIENT_INTERFACE_HPP__

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <vector>
#include "open62541pp/open62541pp.h"

namespace ros2_opcua
{
    namespace node_interface
    {
        class NodeOpcUAClientInterface
        {
        public:
            virtual ~NodeOpcUAClientInterface() = default;

            void createClient();
            void connectClient();
            void disconnectClient();
            void createSubscription();
            void createMonitoredItems();
            void readValue();
            void writeValue();
            void browse();
            void callMethod();

            virtual void configure() = 0;
            virtual void activate() = 0;
            virtual void deactivate() = 0;
            virtual void cleanup() = 0;
            virtual void shutdown() = 0;

            private: 
                std::unique_ptr<opcua::Client> client_{nullptr};
                mutable std::shared_mutex client_mutex_{};
                mutable std::shared_mutex rw_mutex_{};
        };
    }
}


#endif //NODE_OPCUA_CLIENT_INTERFACE_HPP__
