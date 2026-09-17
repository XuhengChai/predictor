#include "zf_hal_sync/hal_sync_data_node.h"

#include <cv_bridge/cv_bridge.h>

#include "rclcpp_components/register_node_macro.hpp"
#include "zf_framework_transport/transport.h"
#include "zf_global/util/logger.h"
#include "zf_global/zf_global_topic_name.h"

BEGIN_NS_ZF_HAL_SYNC

using namespace std::chrono_literals;

HalDataSyncNode::HalDataSyncNode(const rclcpp::NodeOptions &options,
                                 std::string name)
    : Base(options, name) {
  rclcpp::QoS video_qos(50);
  video_qos.keep_last(50);
  video_qos.best_effort();
  video_qos.durability_volatile();
  //   topicName = "/" + m_strCamPrefix + gk_camImageRaw; // /camera/image_raw
  // /hal/from_sync_datas
  std::string topicName = gk_halFromSyncData;  // /hal/from_sync_datas
  m_pubSyncData = this->create_publisher<can_msgs::msg::RadarCamDatas>(
      topicName, video_qos);

  rclcpp::QoS status_qos(50);
  status_qos.keep_last(50);
  status_qos.reliable();
  status_qos.durability_volatile();
  // m_pubNodeStatus = this->create_publisher<std_msgs::msg::UInt32>(
  //     gk_halNodeStatusSync, status_qos); //"/node_status/sync"
  // m_timerNodeSataus = create_wall_timer(100ms, [this]()
  //                                       {
  //   m_pubNodeStatus->publish(m_iSyncStatusMsg);
  //   for (size_t i = 0; i < m_vNodeSataus.size(); i++) {
  //     if (m_vNodeSataus[i] != static_cast<uint32_t>(ENodeStatus::READY)) {
  //       return;
  //     }
  //   }
  //   m_iMissingCount++; });
  m_pubNodeStatus =
      this->create_publisher<NodeStateMsg>("/can_state", status_qos);
  m_iSyncNodeStatusMsg.node_state = static_cast<uint8_t>(ENodeStatus::INIT);
  m_timerNodeSataus = create_wall_timer(100ms, [this]() {
    m_iSyncNodeStatusMsg.watchdog_signal =
        !m_iSyncNodeStatusMsg.watchdog_signal;
    m_pubNodeStatus->publish(m_iSyncNodeStatusMsg);
    for (size_t i = 0; i < m_vNodeSataus.size(); i++) {
      if (m_vNodeSataus[i] != static_cast<uint32_t>(ENodeStatus::READY)) {
        return;
      }
    }
    m_iMissingCount++;
  });
  uint32_t camIdxList[] = {1, 4};
  for (uint32_t i = 0; i < m_strCamPrefix.size(); i++) {
    ImgShmTrans trans;
    trans.camName = m_strCamPrefix.at(i);
    trans.camIdx = camIdxList[i];
    m_mapImgTrans[i] = trans;
    InitReceiver(i);
    InitTransmit(i);
    LOG_DEBUG() << "Cam topicName " << m_strCamPrefix.at(i);
    // rclcpp::Time curr_stamp = this->get_clock()->now();
    // m_dLastTime[i] = curr_stamp.nanoseconds() * 1.0e-9;
  }
}

HalDataSyncNode::~HalDataSyncNode() {}

void HalDataSyncNode::PubSyncData5(const can_msgs::msg::CanDatas &radar,
                                   const CamDataType &img,
                                   const can_msgs::msg::CanDatas &r2,
                                   const can_msgs::msg::CanDatas &r3,
                                   const CamDataType &img2) {
  // std::cout << "time radar 5_CHILD:  "
  //           << "[" << radar.header.stamp.sec << "." << std::setw(9)
  //           << std::setfill('0') << radar.header.stamp.nanosec << "], "
  //           << "[" << img.timestamp << "], "
  //           << "[" << img2.timestamp << "], " << std::endl;
  // can_msgs::msg::RadarCamDatas data;
  // data.header = radar.header;
  // data.radar_datas = radar.msg_datas;
  // data.img = img;
  // data.vehicle_datas = r2.msg_datas;
  // data.ipm_datas = r3.msg_datas;
  // m_pubSyncData->publish(data);
  Base::PubSyncData5(radar, img, r2, r3, img2);
}

void HalDataSyncNode::PubSyncData4(const can_msgs::msg::CanDatas &radar,
                                   const CamDataType &img,
                                   const can_msgs::msg::CanDatas &r2,
                                   const can_msgs::msg::CanDatas &r3) {
  // std::cout << "time radar 4:  "
  //           << "[" << radar.header.stamp.sec << "." << std::setw(9)
  //           << std::setfill('0') << radar.header.stamp.nanosec << "], "
  //           << "[" << img.header.stamp.sec << "." << std::setw(9)
  //           << std::setfill('0') << img.header.stamp.nanosec << "], "
  //           << std::endl;
  can_msgs::msg::RadarCamDatas data;
  data.header = radar.header;
  data.radar_datas = radar.msg_datas;
  // data.img = img;
  data.vehicle_datas = r2.msg_datas;
  data.ipm_datas = r3.msg_datas;
  m_pubSyncData->publish(data);
  Base::PubSyncData4(radar, img, r2, r3);
}

void HalDataSyncNode::PubSyncData3(const can_msgs::msg::CanDatas &radar,
                                   const CamDataType &img,
                                   const can_msgs::msg::CanDatas &r2) {
  // std_msgs::msg::Header head;
  // head.stamp = this->now();
  // std::cout << "time radar 3: "
  //           << "[" << radar.header.stamp.sec << "." << std::setw(9) <<
  //           std::setfill('0') << radar.header.stamp.nanosec
  //           << "], "
  //           << "[" << head.stamp.sec << "." << std::setw(9) <<
  //           std::setfill('0') << head.stamp.nanosec
  //           << "], "
  //           << std::endl;
  can_msgs::msg::RadarCamDatas data;
  data.header = radar.header;
  data.radar_datas = radar.msg_datas;
  // data.img = img;
  switch (m_eSyncPolicy) {
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
  Base::PubSyncData3(radar, img, r2);
}

// void HalDataSyncNode::PubSyncData2(const CamDataType &img, const
// can_msgs::msg::CanDatas &radar)
void HalDataSyncNode::PubSyncData2(const can_msgs::msg::CanDatas &radar,
                                   const CamDataType &img) {
  // std::cout << "time radar: "
  //           << "[" << radar.header.stamp.sec << "." << std::setw(9) <<
  //           std::setfill('0') << radar.header.stamp.nanosec
  //           << "], "
  //           << "[" << img.header.stamp.sec << "." << std::setw(9) <<
  //           std::setfill('0') << img.header.stamp.nanosec
  //           << "], "
  //           << std::endl;
  can_msgs::msg::RadarCamDatas data;
  data.header = radar.header;
  data.radar_datas = radar.msg_datas;
  // data.img = img;
  m_pubSyncData->publish(data);
  Base::PubSyncData2(radar, img);
}

bool HalDataSyncNode::MakeDir(const can_msgs::msg::CanDatas &radar,
                              const CamDataType &img,
                              const can_msgs::msg::CanDatas &r2,
                              const can_msgs::msg::CanDatas &r3,
                              const CamDataType &img2) {
  std::stringstream ss;
  ss << radar.header.stamp.sec << "_" << std::setw(9) << std::setfill('0')
     << radar.header.stamp.nanosec;
  // Create directory
  std::string headStamp = ss.str();
  std::string dir = LOGGER::gk_loggerSyncFilePath + headStamp + "/";

  auto fileName = "pack_5G4T.log";
  LOGGER::FileLogger LOG_FILE_SYNC;
  auto folderName = LOG_FILE_SYNC.CreateFolder(dir);
  LOG_FILE_SYNC.SetFileName(fileName, dir.c_str());
  for (auto &data : radar.msg_datas) {
    LOG_FILE_SYNC() << LogString(data);
  }
  // LOG_FILE_SYNC() << "----end";

  LOGGER::FileLogger LOG_FILE_PROCESS;
  LOG_FILE_PROCESS.SetFileName("pre.log", dir.c_str());
  auto dataStr = m_pRadarPreprocess5G4T->ProcessData(radar);
  // LOG_DEBUG() << dataStr.c_str();
  LOG_FILE_PROCESS() << dataStr;

  switch (m_eSyncPolicy) {
    case ESyncPolicy::SYNC_CAMERA_5G4T:
      break;
    case ESyncPolicy::SYNC_CAMERA_5G4T_VEHICLE:
      break;
    case ESyncPolicy::SYNC_CAMERA_5G4T_IPM: {
      LOGGER::FileLogger LOG_FILE_PROCESS_IPM;
      LOG_FILE_PROCESS_IPM.SetFileName("pre_ipm.log", dir.c_str());
      auto dataIpmStr = m_pRadarPreprocessIPM->ProcessData(r2);
      // LOG_DEBUG() << dataIpmStr.c_str();
      LOG_FILE_PROCESS_IPM() << dataIpmStr;
      break;
    }
    case ESyncPolicy::SYNC_ALL:
    case ESyncPolicy::SYNC_ALL_TWO_CAMS: {
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
      if (m_eSyncPolicy == ESyncPolicy::SYNC_ALL_TWO_CAMS) {
        headStamp = img2.timestamp;
        ss.clear();
        ss.str("");
        ss << folderName << m_mapImgTrans[1].camName << "_" << headStamp
           << ".jpg";
        // std::string imgName =
        //     folderName + m_mapImgTrans[1].camName + "_" + headStamp + ".jpg";
        cv::imwrite(ss.str(), img2.img,
                    {cv::IMWRITE_JPEG_QUALITY, m_iImgQuality});
      }
      break;
    }
    default:
      break;
  }

  // Create directory
  headStamp = img.timestamp;
  ss.clear();
  ss.str("");
  ss << folderName << m_mapImgTrans[0].camName << "_" << headStamp << ".jpg";
  // std::string imgName =
  //     folderName + m_mapImgTrans[1].camName + "_" + headStamp + ".jpg";
  cv::imwrite(ss.str(), img.img, {cv::IMWRITE_JPEG_QUALITY, m_iImgQuality});

  return true;
}

void HalDataSyncNode::NodeStatusCallback(
    const std_msgs::msg::UInt32::SharedPtr msg, int index) {
  Base::NodeStatusCallback(msg, index);
  if (m_iMissingCount > 10) {
    m_iMissingCount = 10;
    m_iSyncStatusMsg.data = static_cast<uint32_t>(ENodeStatus::ERROR);
  }
  m_iSyncNodeStatusMsg.node_state = static_cast<uint8_t>(m_iSyncStatusMsg.data);
}

void HalDataSyncNode::InitTransmit(const uint32_t &idx) {
  NS_ZF_FRAMEWORK::RoleAttributes attr;
  attr.SetHostIp(NS_ZF::GlobalData::Instance().HostIp());
  attr.SetChannelName(gk_channelDetToUi + m_mapImgTrans[idx].camName);
  NS_ZF_FRAMEWORK::Identity id;
  attr.SetId(id.HashValue());
  m_mapImgTrans[idx].transmitterImg =
      NS_ZF_FRAMEWORK::Transport::Instance()
          .CreateTransmitter<NS_ZF_FRAMEWORK::MsgBase>(
              attr, NS_ZF_FRAMEWORK::OptionalMode::SHM);
}

void HalDataSyncNode::InitReceiver(const uint32_t &idx) {
  NS_ZF_FRAMEWORK::RoleAttributes attr;
  attr.SetHostIp(NS_ZF::GlobalData::Instance().HostIp());
  attr.SetChannelName(gk_channelCamToDet + m_mapImgTrans[idx].camName);
  NS_ZF_FRAMEWORK::Identity id;
  attr.SetId(id.HashValue());

  auto listener = [this, idx](
                      const std::shared_ptr<NS_ZF_FRAMEWORK::MsgBase> &msg,
                      const NS_ZF_FRAMEWORK::MessageInfo &msg_info,
                      const NS_ZF_FRAMEWORK::RoleAttributes &attr) {
    (void)msg_info;
    // struct timespec ts;
    // clock_gettime(CLOCK_REALTIME, &ts);

    // LOG_INFO() << "-------I heared: " << msg->Data().size()
    //            << " DELAY: " << (nanoseconds - msg->TimeStamp()) / 1000LL
    //            << " send tmp: " << msg->TimeStamp() / 1000000000LL
    //            << "." << msg->TimeStamp() % 1000000000LL
    //            //  << " recv tmp: " << ts.tv_sec << "." << ts.tv_nsec;
    //            << " recv tmp: " << nanoseconds / 1000000000LL
    //            << "." << nanoseconds % 1000000000LL;
    // Decode data into Mat
    // std::chrono::steady_clock::time_point begin =
    // std::chrono::steady_clock::now();
    rclcpp::Time curr_stamp = this->get_clock()->now();
    long long nanoseconds = curr_stamp.nanoseconds();

    char *raw_memory = (const_cast<char *>(msg->Data().data()));
    cv::Mat decodedImage(1080, 1920, CV_8UC3, raw_memory);
    cv::cvtColor(decodedImage, decodedImage, cv::COLOR_BGR2RGB);
    CamDataType imgWithTime;
    imgWithTime.img = decodedImage.clone();
    std::ostringstream convert;
    convert << msg->TimeStamp() / 1000000000LL << "." << std::setw(9)
            << std::setfill('0') << msg->TimeStamp() % 1000000000LL;
    imgWithTime.timestamp = convert.str();

    double h_time = msg->TimeStamp() * 1.0e-9;
    // double elapsed_time = h_time - m_dLastTime[idx];
    // double fpsRos = 1.0 / elapsed_time;
    // m_dLastTime[idx] = h_time;
    switch (m_eSyncPolicy) {
      case ESyncPolicy::SYNC_CAMERA_5G4T:
        m_timeSync2->PushMsg(imgWithTime, h_time, m_mapImgTrans[idx].camIdx);
        break;
      case ESyncPolicy::SYNC_CAMERA_5G4T_VEHICLE:
      case ESyncPolicy::SYNC_CAMERA_5G4T_IPM:
        m_timeSync3->PushMsg(imgWithTime, h_time, m_mapImgTrans[idx].camIdx);

        break;
      case ESyncPolicy::SYNC_ALL:
        m_timeSync4->PushMsg(imgWithTime, h_time, m_mapImgTrans[idx].camIdx);
        // LOG_DEBUG() << idx << " PushMsg ImageDataCallback: "
        //             << imgWithTime.timestamp << ", "
        //             << " DELAY: " << (nanoseconds - msg->TimeStamp()) /
        //             1000LL << ", "
        //             << std::to_string(elapsed_time).c_str()
        //             << " fps: " << std::floor(fpsRos * 100 + 0.5) / 100;
        break;
      case ESyncPolicy::SYNC_ALL_TWO_CAMS:
        m_timeSync5->PushMsg(imgWithTime, h_time, m_mapImgTrans[idx].camIdx);
        break;
      default:
        break;
    }

    // m_mapImgTrans[idx].transmitterImg->Transmit(msg);

    // rclcpp::Time curr_stamp = this->get_clock()->now();
    // long long nanoseconds = curr_stamp.nanoseconds();
    // std::cout << idx
    //     // << "Time imdecode = " <<
    //     // std::chrono::duration_cast<std::chrono::microseconds>(end -
    //     // begin).count() << "[µs]"
    //     // << "I heared: " << buffer.size()
    //     << " DELAY: " << (nanoseconds - msg->TimeStamp()) / 1000LL
    //     << " send tmp: " << msg->TimeStamp() / 1000000000LL << "."
    //     << msg->TimeStamp() % 1000000000LL
    //     << " recv tmp: " << nanoseconds / 1000000000LL << "."
    //     << nanoseconds % 1000000000LL << ", "
    //     << std::to_string(elapsed_time).c_str()
    //     << " fps: " << std::floor(fpsRos * 100 + 0.5) / 100 << std::endl;
    // ;

    // std::string total_name;
    // std::ostringstream convert;
    // // Extract time stamp
    // convert << "/home/nvidia/Pictures/camera/";
    // convert << msg->TimeStamp() / 1000000000LL
    //         << "_" << msg->TimeStamp() % 1000000000LL;
    // convert << ".jpg";
    // total_name = convert.str();
    // // LOG_INFO() << "-------I heared imwrite: "  << total_name.c_str();
    // cv::imwrite(total_name, decodedImage, {cv::IMWRITE_JPEG_QUALITY, 50});
  };

  m_mapImgTrans[idx].recvShmImg =
      NS_ZF_FRAMEWORK::Transport::Instance()
          .CreateReceiver<NS_ZF_FRAMEWORK::MsgBase>(
              attr, listener, NS_ZF_FRAMEWORK::OptionalMode::SHM);
}

END_NS_ZF_HAL_SYNC

RCLCPP_COMPONENTS_REGISTER_NODE(NS_ZF::NS_DRIVER::NS_HAL_SYNC::HalDataSyncNode)