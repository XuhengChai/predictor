#include "zf_hal_sync/hal_sync_tracking_can_node.h"

#include "rclcpp_components/register_node_macro.hpp"
#include "zf_global/util/logger.h"
#include "zf_global/zf_global_topic_name.h"

BEGIN_NS_ZF_HAL_SYNC

using namespace std::chrono_literals;

SyncTrackingCanNode::SyncTrackingCanNode(const rclcpp::NodeOptions &options,
                                         std::string name)
    : Base(options, name)
{
  switch (m_eSyncPolicy)
  {
  case ESyncPolicy::SYNC_CAMERA_5G4T:
    m_timeSync2->SetDataPeriodI(1, 0.06);
    // m_timeSync2->SetMasterSensor(1);
    break;
  case ESyncPolicy::SYNC_CAMERA_5G4T_VEHICLE:
  case ESyncPolicy::SYNC_CAMERA_5G4T_IPM:
    m_timeSync3->SetDataPeriodI(1, 0.06);
    break;
  case ESyncPolicy::SYNC_ALL:
    m_timeSync4->SetDataPeriodI(1, 0.06);
    break;
  case ESyncPolicy::SYNC_ALL_TWO_CAMS:
    m_timeSync5->SetDataPeriodI(1, 0.06);
    m_timeSync5->SetDataPeriodI(4, 0.06);
    break;
  default:
    break;
  }
  rclcpp::QoS video_qos(50);
  video_qos.keep_last(50);
  video_qos.best_effort();
  video_qos.durability_volatile();
  std::string topicName = "/tracked_objects"; // /camera/image_raw
  std::function<void(const CamTrackingDataType::ConstSharedPtr)> cbk =
      std::bind(&HalDataSyncBase::ImageDataCallback, this,
                std::placeholders::_1, 1);
  m_subImageData =
      this->create_subscription<CamTrackingDataType>(topicName, video_qos, cbk);
  LOG_DEBUG() << "Cam topicName " << topicName.c_str();
  topicName = gk_halFromSyncData; // /hal/from_sync_datas
  m_pubSyncData = this->create_publisher<can_msgs::msg::RadarTrackingDatas>(
      topicName, video_qos);

  rclcpp::QoS status_qos(50);
  status_qos.keep_last(50);
  status_qos.reliable();
  status_qos.durability_volatile();
  // m_pubNodeStatus =
  // this->create_publisher<std_msgs::msg::UInt32>(gk_halNodeStatusSync,
  // status_qos); //"/node_status/sync"
  m_pubNodeStatus =
      this->create_publisher<NodeStateMsg>("/can_state", status_qos);
  m_iSyncNodeStatusMsg.node_state = static_cast<uint8_t>(ENodeStatus::INIT);
  m_timerNodeSataus = create_wall_timer(100ms, [this]()
                                        {
                                          m_iSyncNodeStatusMsg.watchdog_signal =
                                              !m_iSyncNodeStatusMsg.watchdog_signal;
                                          m_pubNodeStatus->publish(m_iSyncNodeStatusMsg);
                                          for (size_t i = 0; i < m_vNodeSataus.size(); i++)
                                          {
                                            if (m_vNodeSataus[i] != static_cast<uint32_t>(ENodeStatus::READY))
                                            {
                                              return;
                                            }
                                          }
                                          // m_iMissingCount++;
                                        });
}

SyncTrackingCanNode::~SyncTrackingCanNode() {}

void SyncTrackingCanNode::PubSyncData4(const can_msgs::msg::CanDatas &radar,
                                       const CamTrackingDataType &trackData,
                                       const can_msgs::msg::CanDatas &r2,
                                       const can_msgs::msg::CanDatas &r3)
{
  std::cout << "time radar 4:  "
            << "[" << radar.header.stamp.sec << "." << std::setw(9)
            << std::setfill('0') << radar.header.stamp.nanosec << "], "
            << "[" << trackData.header.stamp.sec << "." << std::setw(9)
            << std::setfill('0') << trackData.header.stamp.nanosec << "], "
            << std::endl;
  can_msgs::msg::RadarTrackingDatas data;
  data.header = radar.header;
  data.radar_datas = radar.msg_datas;
  data.array = trackData.array;
  data.vehicle_datas = r2.msg_datas;
  data.ipm_datas = r3.msg_datas;

  m_pubSyncData->publish(data);
  Base::PubSyncData4(radar, trackData, r2, r3);
}

void SyncTrackingCanNode::PubSyncData3(const can_msgs::msg::CanDatas &radar,
                                       const CamTrackingDataType &trackData,
                                       const can_msgs::msg::CanDatas &r2)
{
  can_msgs::msg::RadarTrackingDatas data;
  data.header = radar.header;
  data.radar_datas = radar.msg_datas;
  data.array = trackData.array;
  switch (m_eSyncPolicy)
  {
  case ESyncPolicy::SYNC_CAMERA_5G4T_VEHICLE:
    data.vehicle_datas = r2.msg_datas;
    break;
  case ESyncPolicy::SYNC_CAMERA_5G4T_IPM:
    data.ipm_datas = r2.msg_datas;
    break;
  default:
    break;
  }
  m_pubSyncData->publish(data);
  Base::PubSyncData3(radar, trackData, r2);
}

void SyncTrackingCanNode::PubSyncData2(const can_msgs::msg::CanDatas &radar,
                                       const CamTrackingDataType &trackData)
{
  // std::cout << "time radar: "
  //           << "[" << radar.header.stamp.sec << "." << std::setw(9) <<
  //           std::setfill('0') << radar.header.stamp.nanosec
  //           << "], "
  //           << "[" << trackData.header.stamp.sec << "." << std::setw(9) <<
  //           std::setfill('0') << trackData.header.stamp.nanosec
  //           << "], "
  //           << std::endl;
  can_msgs::msg::RadarTrackingDatas data;
  data.header = radar.header;
  data.radar_datas = radar.msg_datas;
  data.array = trackData.array;
  m_pubSyncData->publish(data);
  Base::PubSyncData2(radar, trackData);
}

bool SyncTrackingCanNode::MakeDir(const can_msgs::msg::CanDatas &radar,
                                  const CamTrackingDataType &trackData,
                                  const can_msgs::msg::CanDatas &r2,
                                  const can_msgs::msg::CanDatas &r3,
                                  const CamTrackingDataType &trackData2)
{
  std::stringstream ss;
  ss << radar.header.stamp.sec << "_" << std::setw(9) << std::setfill('0')
     << radar.header.stamp.nanosec;
  // Create directory
  std::string headStamp = ss.str();
  std::string dir = LOGGER::gk_loggerSyncFilePath + headStamp + "/";

  auto fileName = "pack_5G4T.log";
  LOGGER::FileLogger LOG_FILE_SYNC;
  LOG_FILE_SYNC.CreateFolder(dir);
  LOG_FILE_SYNC.SetFileName(fileName, dir.c_str());
  for (auto &data : radar.msg_datas)
  {
    LOG_FILE_SYNC() << LogString(data);
  }
  // LOG_FILE_SYNC() << "----end";

  LOGGER::FileLogger LOG_FILE_PROCESS;
  LOG_FILE_PROCESS.SetFileName("pre.log", dir.c_str());
  auto dataStr = m_pRadarPreprocess5G4T->ProcessData(radar);
  // LOG_DEBUG() << dataStr.c_str();
  LOG_FILE_PROCESS() << dataStr;

  switch (m_eSyncPolicy)
  {
  case ESyncPolicy::SYNC_CAMERA_5G4T:
    break;
  case ESyncPolicy::SYNC_CAMERA_5G4T_VEHICLE:
    break;
  case ESyncPolicy::SYNC_CAMERA_5G4T_IPM:
  {
    LOGGER::FileLogger LOG_FILE_PROCESS_IPM;
    LOG_FILE_PROCESS_IPM.SetFileName("pre_ipm.log", dir.c_str());
    auto dataIpmStr = m_pRadarPreprocessIPM->ProcessData(r2);
    // LOG_DEBUG() << dataIpmStr.c_str();
    LOG_FILE_PROCESS_IPM() << dataIpmStr;
    break;
  }
  case ESyncPolicy::SYNC_ALL:
  {
    LOGGER::FileLogger LOG_FILE_PROCESS_VEHICLE;
    LOG_FILE_PROCESS_VEHICLE.SetFileName("pre_ego.log", dir.c_str());
    auto dataEgoStr = m_pRadarPreprocessVehicle->ProcessData(r2);
    // LOG_DEBUG() << dataEgoStr.c_str();
    LOG_FILE_PROCESS_VEHICLE() << dataEgoStr;

    LOGGER::FileLogger LOG_FILE_PROCESS_IPM;
    LOG_FILE_PROCESS_IPM.SetFileName("pre_ipm.log", dir.c_str());
    auto dataIpmStr = m_pRadarPreprocessIPM->ProcessData(r3);
    // LOG_DEBUG() << dataIpmStr.c_str();
    LOG_FILE_PROCESS_IPM() << dataIpmStr;
    break;
  }
  default:
    break;
  }

  ss.clear();
  ss.str("");
  ss << trackData.header.stamp.sec << "." << std::setw(9) << std::setfill('0')
     << trackData.header.stamp.nanosec << ",";
  // Create directory
  // headStamp = ss.str();
  // std::string imgName = dir + headStamp + ".txt";

  if (trackData.array.layout.dim.size())
  {
    uint32_t rows = trackData.array.layout.dim[0].size;
    uint32_t cols = trackData.array.layout.dim[1].size;
    ss << rows << "," << cols << "\r\n";
    for (size_t i = 0; i < rows; ++i)
    {
      for (size_t j = 0; j < cols; ++j)
      {
        ss << trackData.array.data[i * cols + j] << ",";
      }
      ss << "\r\n";
    }
  }
  LOGGER::FileLogger LOG_FILE_CAM_TRACKING_DATA;
  LOG_FILE_CAM_TRACKING_DATA.SetFileName("pre_cam.log", dir.c_str());
  // LOG_DEBUG() << dataIpmStr.c_str();
  LOG_FILE_CAM_TRACKING_DATA() << ss.str();
}

void SyncTrackingCanNode::NodeStatusCallback(
    const std_msgs::msg::UInt32::SharedPtr msg, int index)
{
  m_iSyncNodeStatusMsg.header.stamp = this->now();
  Base::NodeStatusCallback(msg, index);
  m_iSyncNodeStatusMsg.node_state = static_cast<uint8_t>(m_iSyncStatusMsg.data);
  // if (m_iMissingCount < 10)
  // {
  //   m_iSyncStatusMsg.node_state = static_cast<uint32_t>(ENodeStatus::READY);
  // }
  // else
  // {
  //   m_iSyncStatusMsg.node_state = static_cast<uint32_t>(ENodeStatus::ERROR);
  // }
}

END_NS_ZF_HAL_SYNC

RCLCPP_COMPONENTS_REGISTER_NODE(
    NS_ZF::NS_DRIVER::NS_HAL_SYNC::SyncTrackingCanNode)