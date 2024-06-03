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

            virtual void init() = 0;
            virtual void configure() = 0;
            virtual void activate() = 0;
            virtual void deactivate() = 0;
            virtual void cleanup() = 0;
            virtual void shutdown() = 0;
            
        };
    }
}


#endif //NODE_OPCUA_CLIENT_INTERFACE_HPP__
