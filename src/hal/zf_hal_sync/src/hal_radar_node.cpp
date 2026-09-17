// #include <chrono>
#include "zf_hal_sync/hal_radar_node.h"

#include "rclcpp_components/register_node_macro.hpp"

#include "zf_global/zf_global_topic_name.h"
#include "zf_global/util/logger.h"
#include "zf_hal_can_driver/byte.h"

BEGIN_NS_ZF_HAL_SYNC

LOGGER::FileLogger LOG_FILE_PACKER;

using namespace can_msgs;
// using namespace std::chrono_literals;

HalRadarPackNode::HalRadarPackNode(const rclcpp::NodeOptions &options, std::string name) : Node(name, options)
{
    m_bEnableDebug = this->declare_parameter<bool>("enable_debug", false);

    LOG_FILE_PACKER.SetFileName("build_at_" __DATE__ "_packer.log");
    rclcpp::QoS video_qos(500);
    video_qos.keep_last(500);
    video_qos.best_effort();
    video_qos.durability_volatile();

    RadarPackPolicyPtr radarAC1000T = std::make_shared<RadarAC1000TPack>("can0", ERadarType::RADAR_AC1000T);
    RadarPackPolicyPtr radar5G4T = std::make_shared<Radar5G4TPack>("can1", ERadarType::RADAR_5G4T);
    RadarPackPolicyPtr radarIpm = std::make_shared<RadarIPMPack>("pcan0", ERadarType::RADAR_IPM);
    // m_mapRadarPackers[radarAC1000T->GetRardarType()] = radarAC1000T;
    m_mapRadarPackers[radarIpm->GetRardarType()] = radarIpm;
    m_mapRadarPackers[radar5G4T->GetRardarType()] = radar5G4T;
    std::string subTopicName = "";
    std::string pubTopicName = "";
    for (auto &it : m_mapRadarPackers)
    {
        auto tPack = it.second;
        ERadarType type = static_cast<ERadarType>(tPack->GetRardarType());
        subTopicName = "/" + tPack->GetPrefix() + gk_halFromCanMsgData; // from_can_msg_data
        // auto callback = std::bind(&HalRadarPackNode::RadarDataCallback, this, std::placeholders::_1, RADAR_5G4T);
        // LOG_DEBUG() << "subTopicName " << subTopicName.c_str();
        std::function<void(const can_msgs::msg::CanMsgData::ConstSharedPtr)> cbk =
            std::bind(&HalRadarPackNode::RadarDataCallback, this, std::placeholders::_1, type);
        m_subCanMsgData[type] = this->create_subscription<msg::CanMsgData>(
            subTopicName, video_qos, cbk);
        // LOG_INFO() << "subTopic type " << type;

        // m_subCanMsgData[type] = this->create_subscription<msg::CanMsgData>(
        //     subTopicName, video_qos,
        //     [this, type](const can_msgs::msg::CanMsgData::SharedPtr msg)
        //     {
        //         RadarDataCallback(msg, type);
        //     });
        pubTopicName = "/" + tPack->GetPrefix() + gk_halFromPackedRadar; // from_sync_radar_data
        m_pubRadarData[type] = this->create_publisher<msg::CanDatas>(pubTopicName, video_qos);
    }
    // m_timerCan1 = create_wall_timer(
    //     60ms, [this]()
    //     {
    //         msg::CanDatas data;
    //         data.header.stamp = this->now();
    //         m_pubRadarData[ERadarType::RADAR_5G4T]->publish(data); });
    // m_timerPcan0 = create_wall_timer(
    //     55ms, [this]()
    //     {
    //         msg::CanDatas data;
    //         data.header.stamp = this->now();
    //         m_pubRadarData[ERadarType::RADAR_IPM]->publish(data); });
}

HalRadarPackNode::~HalRadarPackNode()
{
}

void HalRadarPackNode::RadarDataCallback(const can_msgs::msg::CanMsgData::ConstSharedPtr msg, ERadarType type)
{
    auto tPackPtr = m_mapRadarPackers[type];
    auto tId = tPackPtr->PackedRadarDataCallback(msg);
    // if (type == ERadarType::RADAR_5G4T)
    // {
    //     LOG_FILE_PACKER() << LogString(*msg);
    // }
    if (tId != -1)
    {
        m_pubRadarData[type]->publish(*tPackPtr->GetObjData(tId));
        // if (m_bEnableDebug)
        // {
        //     LogStrings(*tPackPtr->GetObjData(tId));
        // }
        tPackPtr->ResetRadarData(tId);
    }
}

std::string HalRadarPackNode::LogString(const can_msgs::msg::CanMsgData &msgData)
{
    std::stringstream output_stream("");
    std_msgs::msg::Header head;
    head.stamp = this->now();
    output_stream << "[" << head.stamp.sec << "." << std::setw(9) << std::setfill('0') << head.stamp.nanosec
                  << "], "
                  << "[" << msgData.header.stamp.sec << "." << std::setw(9) << std::setfill('0') << msgData.header.stamp.nanosec
                  << "], "
                  << msgData.msg_name << ", " << msgData.msg_pgn
                  << ", " << NS_ZF::NS_DRIVER::NS_CANBUS::Byte::Int2Hex(msgData.msg_id) << ", ";
    for (auto &sig : msgData.sig_datas)
    {
        output_stream << sig.sig_name
                      << ", " << sig.sig_data << ", ";
    }
    return output_stream.str();
}

void HalRadarPackNode::LogStrings(const can_msgs::msg::CanDatas &msgDatas)
{
    for (auto &data : msgDatas.msg_datas)
    {
        LOG_FILE_PACKER() << LogString(data);
    }
    LOG_FILE_PACKER() << "----end";
}

END_NS_ZF_HAL_SYNC

RCLCPP_COMPONENTS_REGISTER_NODE(NS_ZF::NS_DRIVER::NS_HAL_SYNC::HalRadarPackNode)