#ifndef ZF_FRAMEWORK_SYS_MANAGER_H
#define ZF_FRAMEWORK_SYS_MANAGER_H

#include <memory>
#include <string>
#include <cstring>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"

#include "zf_global/in/zf_framework_global.h"
#include "zf_global/common/zf_global_error_code.h"
#include "interface/msg/node_state.hpp"
#include "interface/msg/system_state.hpp"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

enum ESysNodeStatus : uint8_t {
    INITIALIZATION = 10,
    READY = 20,
    READY_WITH_DOWNGRADE = 25,
    ENABLED = 30,
    ENABLED_WITH_DOWNGRADE = 35,
    ACTIVE = 40,
    ACTIVE_WITH_DOWNGRADE = 45,
    ERROR = 60,
};

enum EPriorityState : uint8_t
{
    PRIORITY_ENABLED = 1,
    PRIORITY_DOWNGRADE = 2,
    PRIORITY_ERROR = 3,
    PRIORITY_INIT = 4,
};

enum ESysNodeName : uint8_t
{
    NODE_CAM1 = 1,
    NODE_CAM2 = 2,
    NODE_CAN = 10,
    NODE_SYNC = 20,
    NODE_EGO_STATE = 30,
};

struct NodeInfo
{
    uint8_t watchDogState;
    uint8_t state;
    bool lastWatchDog;
    uint32_t missingCnt;
    uint32_t idx;
}; 

/*
 * Node State:
 * Initialization = 10, Enabled = 30, Downgrade = 50, Error = 60
 */
class SysManager : public rclcpp::Node
{
public:
    using NodeStateMsg = interface::msg::NodeState;
    using NodeStateMsgSubPtr = rclcpp::Subscription<NodeStateMsg>::SharedPtr;
    using SysStateMsg = interface::msg::SystemState;
    using Uint8Vec = std::vector<uint8_t>;

    SysManager(const rclcpp::NodeOptions &, std::string name = "data_sys_manager");
    ~SysManager(){};
private:
    void NodeStatusCallback(const NodeStateMsg::ConstSharedPtr &msg, ESysNodeName flag);
    void PubSystemStatus();
    void UpdateSystemStatus(uint8_t state);
private:
    std::unordered_map<ESysNodeName, NodeStateMsgSubPtr> m_mapSubNodesStatus;
    std::unordered_map<ESysNodeName, NodeInfo> m_mapNodesInfo;
    std::unordered_map<uint8_t, EPriorityState> m_mapStatusPriority;
    rclcpp::Publisher<SysStateMsg>::SharedPtr m_pubSysState;
    SysStateMsg m_msgSysState;
    rclcpp::TimerBase::SharedPtr m_timerPubSysState;
    Uint8Vec m_vWatchDog;
    Uint8Vec m_vNodeState;
};

END_NS_ZF_FRAMEWORK // zf::framework

#endif // ZF_FRAMEWORK_SYS_MANAGER_H
