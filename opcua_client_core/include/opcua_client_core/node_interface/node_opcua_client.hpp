#ifndef NODE_OPCUA_CLIENT_HPP__
#define NODE_OPCUA_CLIENT_HPP__

#include "opcua_client_core/datatypes.hpp"
#include "opcua_client_core/node_interface/node_opcua_client_interface.hpp"
#include "opcua_client_core/type_registry.hpp"
#include <yaml-cpp/yaml.h>

namespace ros2_opcua
{
namespace node_interface
{
struct variableInfo
{
    std::string name;
    uint8_t index;
    std::string type;
    std::string typeID;
    std::string bynaryTypeID;
    std::vector<std::map<std::string, std::string>> elements;
};

template <class NODETYPE> class NodeOpcUAClient : public NodeOpcUAClientInterface
{
  public:
    NodeOpcUAClient(NODETYPE *node) { node_ = node; }

    void init();
    void configure();
    void activate();
    void deactivate();
    void cleanup();
    void shutdown();

    opcua::Client &getClient() { return *opcua_client_; }

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

    std::vector<variableInfo> variables_;

  private:
    void loadVariables(const YAML::Node &config);
    void updateVariables();
};
} // namespace node_interface
} // namespace ros2_opcua

#endif // NODE_OPCUA_CLIENT_HPP__
