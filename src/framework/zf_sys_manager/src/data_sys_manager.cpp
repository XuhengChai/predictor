#include "zf_sys_manager/data_sys_manager.h"

#include <cstdio>
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

#include "zf_global/zf_global_topic_name.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework
using namespace std::chrono_literals;
static const int gk_maxMissingCnt = 5;

SysManager::SysManager(const rclcpp::NodeOptions &options,
                       std::string name) : Node(name, options)

{
  m_mapStatusPriority = {
      {static_cast<uint8_t>(ENodeStatus::INIT), EPriorityState::PRIORITY_INIT},
      {static_cast<uint8_t>(ENodeStatus::READY), EPriorityState::PRIORITY_ENABLED},
      {static_cast<uint8_t>(ENodeStatus::DOWNGRADE), EPriorityState::PRIORITY_DOWNGRADE},
      {static_cast<uint8_t>(ENodeStatus::ERROR), EPriorityState::PRIORITY_ERROR},

  };
  std::vector<std::pair<ESysNodeName, std::string>>
      nodes = {
          {ESysNodeName::NODE_CAM1, "/cam1/node_state"},
          {ESysNodeName::NODE_CAM2, "/cam2/node_state"},
          {ESysNodeName::NODE_SYNC, "/can_state"},
          {ESysNodeName::NODE_EGO_STATE, "/ego_state"},
      };
  rclcpp::QoS video_qos(10);
  video_qos.keep_last(10);
  video_qos.best_effort();
  // video_qos.reliable();
  video_qos.durability_volatile();
  uint32_t idx = 0;
  uint32_t init = static_cast<uint8_t>(ENodeStatus::INIT);
  for (auto &nodeName : nodes)
  {
    std::function<void(const NodeStateMsg::ConstSharedPtr)> cbk =
        std::bind(&SysManager::NodeStatusCallback, this,
                  std::placeholders::_1, nodeName.first);
    m_mapSubNodesStatus[nodeName.first] = this->create_subscription<NodeStateMsg>(
        nodeName.second, video_qos, cbk); // 1.1M
    NodeInfo info = {init, init, false, 0, idx};
    idx++;
    m_mapNodesInfo[nodeName.first] = info;
    m_vWatchDog.push_back(init);
    m_vNodeState.push_back(init);
  }

  // std::string sysStateData = "/data_sys_manager/sys_state";
  m_pubSysState = this->create_publisher<SysStateMsg>(gk_uiDataSystemState, video_qos);
  m_timerPubSysState = create_wall_timer(250ms, std::bind(&SysManager::PubSystemStatus, this));
}

void SysManager::NodeStatusCallback(const NodeStateMsg::ConstSharedPtr &msg, ESysNodeName flag)
{
  //   {ESysNodeName::NODE_CAM1, "/cam1/node_state"},
  // {ESysNodeName::NODE_CAM2, "/cam1/node_state"},
  // {ESysNodeName::NODE_SYNC, "/cam1/node_state"},
  // {ESysNodeName::NODE_EGO_STATE, "/cam1/node_state"},
  m_mapNodesInfo[flag].state = msg->node_state;
  if (m_mapNodesInfo[flag].lastWatchDog == msg->watchdog_signal)
  {
    m_mapNodesInfo[flag].missingCnt++;
    m_mapNodesInfo[flag].watchDogState = static_cast<uint8_t>(ESysNodeStatus::ERROR);
    ;
  }
  else
  {
    if (m_mapNodesInfo[flag].missingCnt > 0)
    {
      m_mapNodesInfo[flag].missingCnt--;
    }
    m_mapNodesInfo[flag].watchDogState = static_cast<uint8_t>(ESysNodeStatus::ENABLED);
  }
  m_mapNodesInfo[flag].lastWatchDog = msg->watchdog_signal;
}

void SysManager::PubSystemStatus()
{
  m_msgSysState.trigger_state_hmi = static_cast<uint8_t>(ESysNodeStatus::ENABLED);
  for (auto &nodeInfo : m_mapNodesInfo)
  {
    nodeInfo.second.missingCnt++;
    auto info = nodeInfo.second;
    m_vNodeState[info.idx] = info.state;
    m_vWatchDog[info.idx] = info.watchDogState;
    if (info.missingCnt > gk_maxMissingCnt)
    {
      UpdateSystemStatus(ESysNodeStatus::ERROR);
      nodeInfo.second.watchDogState = static_cast<uint8_t>(ESysNodeStatus::ERROR);
      nodeInfo.second.missingCnt = gk_maxMissingCnt;
    }
    if (info.state != m_msgSysState.trigger_state_hmi)
    {
      UpdateSystemStatus(info.state);
    }
    if (info.watchDogState != m_msgSysState.trigger_state_hmi)
    {
      UpdateSystemStatus(info.watchDogState);
    }
  }
  m_msgSysState.watchdog_state_list = m_vWatchDog;
  m_msgSysState.node_state_list = m_vNodeState;
  m_msgSysState.watchdog_signal = !m_msgSysState.watchdog_signal;
  m_msgSysState.header.stamp = this->get_clock()->now();
  m_pubSysState->publish(m_msgSysState);
}

void SysManager::UpdateSystemStatus(uint8_t state)
{
  if (m_mapStatusPriority[m_msgSysState.trigger_state_hmi] < m_mapStatusPriority[state])
  {
    m_msgSysState.trigger_state_hmi = state;
  }
}

END_NS_ZF_FRAMEWORK // zf::framework

RCLCPP_COMPONENTS_REGISTER_NODE(zf::framework::SysManager)

    // int main(int argc, char **argv)
    // {
    //   rclcpp::init(argc, argv);
    //   /*创建对应节点的共享指针对象*/
    //   // auto node = std::make_shared<MinimalDepthSubscriber>();
    //   const rclcpp::NodeOptions options;
    //   auto node = std::make_shared<NS_ZF_FRAMEWORK::SysManager>(options, "data_sys_manager");
    //   /* 运行节点，并检测退出信号*/
    //   rclcpp::spin(node);
    //   rclcpp::shutdown();
    //   return 0;
    // }