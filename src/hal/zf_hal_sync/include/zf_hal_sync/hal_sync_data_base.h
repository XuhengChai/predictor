/******************************************************************************
 * Copyright 2023 ZF. All Rights Reserved.
 * Author:
 *        xuheng.chai@zf.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
/**
 * @file
 * @brief Defines the hal datas sync ros node.
 * @return
 */

#ifndef ZF_HAL_DATA_SYNC_BASE_H_
#define ZF_HAL_DATA_SYNC_BASE_H_

#include "can_msgs/msg/can_datas.hpp"
#include "can_msgs/msg/can_msg_data.hpp"
#include "can_msgs/msg/can_sig_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "zf_global/in/hal_sync_global.h"
#include "zf_global/zf_global_topic_name.h"
#include "zf_hal_can_driver/byte.h"
#include "zf_hal_sync/hal_radar_preprocess.h"
#include "zf_hal_sync/sync_policies/time_sync.h"

// /**
//  * @namespace NS_ZF::driver::synchronization
//  */
BEGIN_NS_ZF_HAL_SYNC

static const char gk_canInterfaceIpm[] = "pcan0";
static const float gk_vehiclePeriod = 0.05;

template <class CamDataType>
class HalDataSyncBase : public rclcpp::Node {
 public:
  // using RadarDataMap = std::unordered_map<uint32_t, PackedRadarData>; //
  // status can id 60a, 61a
  //   using CamDataType = sensor_msgs::msg::Image;
  using TimeSyncType5 = TimeSynchronizer<can_msgs::msg::CanDatas, CamDataType,
                                         can_msgs::msg::CanDatas,
                                         can_msgs::msg::CanDatas, CamDataType>;
  using TimeSyncType4 =
      TimeSynchronizer<can_msgs::msg::CanDatas, CamDataType,
                       can_msgs::msg::CanDatas,
                       can_msgs::msg::CanDatas>;  // status can id 60a, 61a
  using TimeSyncType3 = TimeSynchronizer<can_msgs::msg::CanDatas, CamDataType,
                                         can_msgs::msg::CanDatas>;
  // using TimeSyncType2 = TimeSynchronizer<CamDataType,
  // can_msgs::msg::CanDatas>;
  using TimeSyncType2 = TimeSynchronizer<can_msgs::msg::CanDatas, CamDataType>;
  explicit HalDataSyncBase(const rclcpp::NodeOptions &,
                           std::string name = "hal_data_sync_base");
  ~HalDataSyncBase() {};
  void ImageDataCallback(const std::shared_ptr<const CamDataType> msg,
                         int index);

 protected:
  void PackedRadarDataCallback(
      const can_msgs::msg::CanDatas::ConstSharedPtr msg, int index);
  void InitSyncPolicy();
  virtual void PubSyncData5(const can_msgs::msg::CanDatas &radar,
                            const CamDataType &img,
                            const can_msgs::msg::CanDatas &r2,
                            const can_msgs::msg::CanDatas &r3,
                            const CamDataType &img2);
  virtual void PubSyncData4(const can_msgs::msg::CanDatas &radar,
                            const CamDataType &img,
                            const can_msgs::msg::CanDatas &r2,
                            const can_msgs::msg::CanDatas &r3);
  virtual void PubSyncData3(const can_msgs::msg::CanDatas &radar,
                            const CamDataType &img,
                            const can_msgs::msg::CanDatas &r2);
  virtual void PubSyncData2(const can_msgs::msg::CanDatas &radar,
                            const CamDataType &img);
  virtual bool MakeDir(
      const can_msgs::msg::CanDatas &radar, const CamDataType &img,
      const can_msgs::msg::CanDatas &r2 = can_msgs::msg::CanDatas(),
      const can_msgs::msg::CanDatas &r3 = can_msgs::msg::CanDatas(),
      const CamDataType &img2 = CamDataType());
  virtual void NodeStatusCallback(const std_msgs::msg::UInt32::SharedPtr msg,
                                  int index);

 protected:
  std::string LogString(const can_msgs::msg::CanMsgData &msgData);
  void LogStrings(const can_msgs::msg::CanDatas &msgDatas);
  StringVector m_strCamPrefix;
  bool m_bEnableDebug;
  int m_iImgQuality;
  std::shared_ptr<TimeSyncType5> m_timeSync5;
  std::shared_ptr<TimeSyncType4> m_timeSync4;
  std::shared_ptr<TimeSyncType3> m_timeSync3;
  std::shared_ptr<TimeSyncType2> m_timeSync2;
  std::unique_ptr<RadarPreprocess5G4T> m_pRadarPreprocess5G4T;
  std::unique_ptr<RadarPreprocessIpm> m_pRadarPreprocessIPM;
  std::unique_ptr<RadarPreprocess> m_pRadarPreprocessVehicle;
  StringVector m_vCanInterfaces;
  ESyncPolicy m_eSyncPolicy;

  // rclcpp::Subscription<can_msgs::msg::CanDatas>::SharedPtr m_subRadarData;
  std::shared_ptr<rclcpp::Subscription<CamDataType>> m_subImageData;
  //   rclcpp::Publisher<can_msgs::msg::RadarCamDatas>::SharedPtr m_pubSyncData;

  std::vector<rclcpp::Subscription<can_msgs::msg::CanDatas>::SharedPtr>
      m_vSubCanDatas;
  std::vector<rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr>
      m_vSubNodeStatus;
  std::vector<uint32_t> m_vNodeSataus;
  //   rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr m_pubNodeStatus;
  std_msgs::msg::UInt32 m_iSyncStatusMsg;
  rclcpp::TimerBase::SharedPtr m_timerNodeSataus;
  uint32_t m_iMissingCount;
  double m_dLastTimeVehicle;
  double m_dTimeVehicleMax;
  double m_dTimeVehicleMin;
};

template <class CamDataType>
HalDataSyncBase<CamDataType>::HalDataSyncBase(
    const rclcpp::NodeOptions &options, std::string name)
    : Node(name, options),
      m_pRadarPreprocess5G4T(new RadarPreprocess5G4T()),
      m_pRadarPreprocessVehicle(new RadarPreprocess()),
      m_pRadarPreprocessIPM(new RadarPreprocessIpm()) {
  m_dLastTimeVehicle = -1.0;
  m_dTimeVehicleMax = 0.051;  // gk_vehiclePeriod * 1.02;
  m_dTimeVehicleMin = 0.049;  // gk_vehiclePeriod * 0.98;
  m_vCanInterfaces =
      this->declare_parameter<StringVector>("canInterfaces", StringVector({}));
  // StringList canInterfaces = {"can0", "can1", "pcan0"};//order 5G4T, vehicle,
  // and IPM

  // m_strCanInterface = this->declare_parameter<std::string>("can_interface",
  // "can0");
  m_strCamPrefix = this->declare_parameter("cam_ns", StringVector({"cam1"}));
  InitSyncPolicy();
  m_bEnableDebug = this->declare_parameter<bool>("enable_debug", false);
  m_iImgQuality = this->declare_parameter("img_write_quality", 50);
  if (m_iImgQuality < 1 || m_iImgQuality > 100) {
    m_iImgQuality = 50;
  }
  LOG_DEBUG() << "m_iImgQuality is " << m_iImgQuality;

  rclcpp::QoS video_qos(50);
  video_qos.keep_last(50);
  video_qos.best_effort();
  video_qos.durability_volatile();
  std::string topicName = "";
  // m_subRadarData = this->create_subscription<can_msgs::msg::CanDatas>(
  //     topicName, video_qos,
  //     std::bind(&HalDataSyncBase::PackedRadarDataCallback, this,
  //     std::placeholders::_1));

  for (uint32_t i = 0; i < m_vCanInterfaces.size(); i++) {
    topicName = "/" + m_vCanInterfaces.at(i) +
                gk_halFromPackedRadar;  // /hal/from_packed_radar_data
    // if (m_vCanInterfaces.at(i).compare("can0")) {
    //   topicName = "/vehicle_info";
    // }
    int index = i ? i + 1 : i;
    std::function<void(const can_msgs::msg::CanDatas::ConstSharedPtr)> cbk =
        std::bind(&HalDataSyncBase::PackedRadarDataCallback, this,
                  std::placeholders::_1, index);
    auto sub = this->create_subscription<can_msgs::msg::CanDatas>(
        topicName, video_qos, cbk);
    m_vSubCanDatas.push_back(sub);
    LOG_DEBUG() << "m_vCanInterfaces topicName " << topicName.c_str();
  }

  // topicName = "/" + m_strCanInterface + gk_halFromSyncData; //
  // /hal/from_sync_datas
  // topicName = gk_halFromSyncData; // /hal/from_sync_datas
  // m_pubSyncData = this->create_publisher<can_msgs::msg::RadarCamDatas>(
  //     topicName, video_qos);

  StringVector tInterfaces = m_vCanInterfaces;  // order 5G4T, vehicle, and IPM
  // StringVector tInterfaces = {"can1"}; // order 5G4T, vehicle, and IPM
  // StringVector tInterfaces = {"can0", "can1", "pcan0"}; // order 5G4T,
  // vehicle, and IPM
  rclcpp::QoS status_qos(50);
  status_qos.keep_last(50);
  status_qos.reliable();
  status_qos.durability_volatile();
  for (uint32_t i = 0; i < tInterfaces.size(); i++) {
    auto iface = tInterfaces.at(i);
    topicName = "/" + iface + gk_halNodeStatusCanRaw;  // /node_status/can_msg
    std::function<void(const std_msgs::msg::UInt32::SharedPtr)> cbkRaw =
        std::bind(&HalDataSyncBase::NodeStatusCallback, this,
                  std::placeholders::_1, i);
    auto subRaw = this->create_subscription<std_msgs::msg::UInt32>(
        topicName, status_qos, cbkRaw);
    m_vSubNodeStatus.push_back(subRaw);
    m_vNodeSataus.push_back(static_cast<uint32_t>(ENodeStatus::INIT));

    topicName = "/" + iface + gk_halNodeStatusCanMsg;  // /node_status/can_msg
    std::function<void(const std_msgs::msg::UInt32::SharedPtr)> cbkMsg =
        std::bind(&HalDataSyncBase::NodeStatusCallback, this,
                  std::placeholders::_1, tInterfaces.size() + i);
    auto subMsg = this->create_subscription<std_msgs::msg::UInt32>(
        topicName, status_qos, cbkMsg);
    m_vSubNodeStatus.push_back(subMsg);
    m_vNodeSataus.push_back(static_cast<uint32_t>(ENodeStatus::INIT));
  }
  //   m_pubNodeStatus = this->create_publisher<std_msgs::msg::UInt32>(
  //       gk_halNodeStatusSync, status_qos); //"/node_status/sync"
  //   m_timerNodeSataus = create_wall_timer(100ms, [this]()
  //                                         {
  //     m_pubNodeStatus->publish(m_iSyncStatusMsg);
  //     for (size_t i = 0; i < m_vNodeSataus.size(); i++) {
  //       if (m_vNodeSataus[i] != static_cast<uint32_t>(ENodeStatus::READY)) {
  //         return;
  //       }
  //     }
  //     m_iMissingCount++; });
}

template <class CamDataType>
void HalDataSyncBase<CamDataType>::PackedRadarDataCallback(
    const can_msgs::msg::CanDatas::ConstSharedPtr msg, int index) {
  double h_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  // double h_time = rclcpp::Time(msg->header.stamp).seconds();
  switch (m_eSyncPolicy) {
    case ESyncPolicy::SYNC_CAMERA_5G4T:
      m_timeSync2->PushMsg(*msg, h_time, index);  // TODO push msg ptr
      // LOG_DEBUG() << index << " PushMsg ImageDataCallback" << h_time;
      // LOG_DEBUG() << "---PushMsg Radar Callback" <<
      // std::to_string(h_time).c_str();

      break;
    case ESyncPolicy::SYNC_CAMERA_5G4T_VEHICLE:
    case ESyncPolicy::SYNC_CAMERA_5G4T_IPM:
      m_timeSync3->PushMsg(*msg, h_time, index);
      break;
    case ESyncPolicy::SYNC_ALL:
      if (index == 2) {
        if (m_dLastTimeVehicle > 0) {
          double errTime = abs(h_time - m_dLastTimeVehicle);
          if ((errTime > m_dTimeVehicleMax) || (errTime < m_dTimeVehicleMin)) {
            h_time = m_dLastTimeVehicle + gk_vehiclePeriod;
            // LOG_WARN() << "Time revised for vehicle. ";
          }
        }
        m_dLastTimeVehicle = h_time;
      }
      m_timeSync4->PushMsg(*msg, h_time, index);
      break;
    case ESyncPolicy::SYNC_ALL_TWO_CAMS:
      if (index == 2) {
        if (m_dLastTimeVehicle > 0) {
          double errTime = abs(h_time - m_dLastTimeVehicle);
          if ((errTime > m_dTimeVehicleMax) || (errTime < m_dTimeVehicleMin)) {
            h_time = m_dLastTimeVehicle + gk_vehiclePeriod;
            // LOG_WARN() << "Time revised for vehicle. ";
          }
        }
        m_dLastTimeVehicle = h_time;
      }
      m_timeSync5->PushMsg(*msg, h_time, index);
      break;
    default:
      break;
  }
}

template <class CamDataType>
void HalDataSyncBase<CamDataType>::InitSyncPolicy() {
  m_eSyncPolicy = static_cast<ESyncPolicy>(m_vCanInterfaces.size() +
                                           m_strCamPrefix.size() - 1);
  if (m_eSyncPolicy == ESyncPolicy::SYNC_CAMERA_5G4T_VEHICLE) {
    for (auto &name : m_vCanInterfaces) {
      if (!name.compare(gk_canInterfaceIpm) || name.find("eth") != std::string::npos) {
        m_eSyncPolicy = ESyncPolicy::SYNC_CAMERA_5G4T_IPM;
        break;
      }
    }
  }
  LOG_DEBUG() << " m_eSyncPolicy " << m_eSyncPolicy;

  switch (m_eSyncPolicy) {
    case ESyncPolicy::SYNC_CAMERA_5G4T:
      m_timeSync2 = std::make_shared<TimeSyncType2>(25);
      m_timeSync2->RegisterCallback(&HalDataSyncBase::PubSyncData2, this);
      // m_timeSync2->SetMasterSensor(1);
      break;
    case ESyncPolicy::SYNC_CAMERA_5G4T_VEHICLE:
    case ESyncPolicy::SYNC_CAMERA_5G4T_IPM:
      m_timeSync3 = std::make_shared<TimeSyncType3>(25);
      m_timeSync3->RegisterCallback(&HalDataSyncBase::PubSyncData3, this);
      m_timeSync3->SetDataPeriodI(2, 0.055);  // IPM period is 55ms
      // m_timeSync3->SetMasterSensor(1);
      break;
    case ESyncPolicy::SYNC_ALL:
      m_timeSync4 = std::make_shared<TimeSyncType4>(25);
      m_timeSync4->RegisterCallback(&HalDataSyncBase::PubSyncData4, this);
      m_timeSync4->SetDataPeriodI(2, gk_vehiclePeriod);
      m_timeSync4->SetDataPeriodI(3, 0.055);
      // m_timeSync4->SetMasterSensor(1);
      break;
    case ESyncPolicy::SYNC_ALL_TWO_CAMS:
      m_timeSync5 = std::make_shared<TimeSyncType5>(25);
      m_timeSync5->RegisterCallback(&HalDataSyncBase::PubSyncData5, this);
      m_timeSync5->SetDataPeriodI(2, gk_vehiclePeriod);
      m_timeSync5->SetDataPeriodI(3, 0.055);
      m_timeSync5->SetDataPeriodI(4, 0.036);
      // m_timeSync4->SetMasterSensor(1);
      break;
    default:
      break;
  }
}

template <class CamDataType>
void HalDataSyncBase<CamDataType>::ImageDataCallback(
    const std::shared_ptr<const CamDataType> msg, int index) {
  // double h_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  double h_time = rclcpp::Time(msg->header.stamp).seconds();
  // m_timeSync->PushMsg(*msg, h_time, 0);
  switch (m_eSyncPolicy) {
    case ESyncPolicy::SYNC_CAMERA_5G4T:
      m_timeSync2->PushMsg(*msg, h_time, index);
      // LOG_DEBUG() << " PushMsg ImageDataCallback" <<
      // std::to_string(h_time).c_str();

      break;
    case ESyncPolicy::SYNC_CAMERA_5G4T_VEHICLE:
    case ESyncPolicy::SYNC_CAMERA_5G4T_IPM:
      m_timeSync3->PushMsg(*msg, h_time, index);

      break;
    case ESyncPolicy::SYNC_ALL:
      m_timeSync4->PushMsg(*msg, h_time, index);
      break;
    case ESyncPolicy::SYNC_ALL_TWO_CAMS:
      m_timeSync5->PushMsg(*msg, h_time, index);
      break;
    default:
      break;
  }
}

template <class CamDataType>
void HalDataSyncBase<CamDataType>::PubSyncData5(
    const can_msgs::msg::CanDatas &radar, const CamDataType &img,
    const can_msgs::msg::CanDatas &r2, const can_msgs::msg::CanDatas &r3,
    const CamDataType &img2) {
  // std::cout << "time radar 5:  "
  //           << "[" << radar.header.stamp.sec << "." << std::setw(9)
  //           << std::setfill('0') << radar.header.stamp.nanosec << "], "
  //           << "[" << img.header.stamp.sec << "." << std::setw(9)
  //           << std::setfill('0') << img.header.stamp.nanosec << "], "
  //           << "[" << img2.header.stamp.sec << "." << std::setw(9)
  //           << std::setfill('0') << img2.header.stamp.nanosec << "], "
  //           << std::endl;
  //   can_msgs::msg::RadarCamDatas data;
  //   data.header = radar.header;
  //   data.radar_datas = radar.msg_datas;
  //   data.img = img;
  //   data.vehicle_datas = r2.msg_datas;
  //   data.ipm_datas = r3.msg_datas;

  //   m_pubSyncData->publish(data);
  if (m_iMissingCount > 0) {
    m_iMissingCount--;
  }
  if (m_bEnableDebug) {
    auto isSuccess = MakeDir(radar, img, r2, r3, img2);
  }
}

template <class CamDataType>
void HalDataSyncBase<CamDataType>::PubSyncData4(
    const can_msgs::msg::CanDatas &radar, const CamDataType &img,
    const can_msgs::msg::CanDatas &r2, const can_msgs::msg::CanDatas &r3) {
  //   std::cout << "time radar 4:  "
  //             << "[" << radar.header.stamp.sec << "." << std::setw(9)
  //             << std::setfill('0') << radar.header.stamp.nanosec << "], "
  //             << "[" << img.header.stamp.sec << "." << std::setw(9)
  //             << std::setfill('0') << img.header.stamp.nanosec << "], "
  //             << std::endl;
  //   can_msgs::msg::RadarCamDatas data;
  //   data.header = radar.header;
  //   data.radar_datas = radar.msg_datas;
  //   data.img = img;
  //   data.vehicle_datas = r2.msg_datas;
  //   data.ipm_datas = r3.msg_datas;

  //   m_pubSyncData->publish(data);
  if (m_iMissingCount > 0) {
    m_iMissingCount--;
  }
  if (m_bEnableDebug) {
    auto isSuccess = MakeDir(radar, img, r2, r3);
  }
}

template <class CamDataType>
void HalDataSyncBase<CamDataType>::PubSyncData3(
    const can_msgs::msg::CanDatas &radar, const CamDataType &img,
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
  //   can_msgs::msg::RadarCamDatas data;
  //   data.header = radar.header;
  //   data.radar_datas = radar.msg_datas;
  //   data.img = img;
  //   switch (m_eSyncPolicy)
  //   {
  //   case ESyncPolicy::SYNC_CAMERA_5G4T_VEHICLE:
  //     data.vehicle_datas = r2.msg_datas;
  //     break;
  //   case ESyncPolicy::SYNC_CAMERA_5G4T_IPM:
  //     data.ipm_datas = r2.msg_datas;
  //     break;
  //   default:
  //     break;
  //   }
  //   m_pubSyncData->publish(data);
  if (m_iMissingCount > 0) {
    m_iMissingCount--;
  }
  if (m_bEnableDebug) {
    auto isSuccess = MakeDir(radar, img, r2);
  }
}

template <class CamDataType>
void HalDataSyncBase<CamDataType>::PubSyncData2(
    const can_msgs::msg::CanDatas &radar, const CamDataType &img) {
  // std::cout << "time radar: "
  //           << "[" << radar.header.stamp.sec << "." << std::setw(9) <<
  //           std::setfill('0') << radar.header.stamp.nanosec
  //           << "], "
  //           << "[" << img.header.stamp.sec << "." << std::setw(9) <<
  //           std::setfill('0') << img.header.stamp.nanosec
  //           << "], "
  //           << std::endl;
  //   can_msgs::msg::RadarCamDatas data;
  //   data.header = radar.header;
  //   data.radar_datas = radar.msg_datas;
  //   data.img = img;
  //   m_pubSyncData->publish(data);
  if (m_iMissingCount > 0) {
    m_iMissingCount--;
  }
  if (m_bEnableDebug) {
    auto isSuccess = MakeDir(radar, img);
  }
}

template <class CamDataType>
bool HalDataSyncBase<CamDataType>::MakeDir(const can_msgs::msg::CanDatas &radar,
                                           const CamDataType &img,
                                           const can_msgs::msg::CanDatas &r2,
                                           const can_msgs::msg::CanDatas &r3,
                                           const CamDataType &img2) {}

template <class CamDataType>
void HalDataSyncBase<CamDataType>::NodeStatusCallback(
    const std_msgs::msg::UInt32::SharedPtr msg, int index) {
  ErrorCode errCode = static_cast<ErrorCode>(msg->data);
  switch (errCode) {
    case ErrorCode::INIT:
    case ErrorCode::READY:
    case ErrorCode::DOWNGRADE:
    case ErrorCode::ERROR:
      m_vNodeSataus[index] = msg->data;
      break;
    case ErrorCode::OK:
      m_vNodeSataus[index] = static_cast<uint32_t>(ENodeStatus::READY);
      break;
    default:
      std::string codeStr =
          NS_ZF::NS_DRIVER::NS_CANBUS::Byte::Int2Hex(msg->data);
      std::istringstream reader(codeStr.substr(0, 2));
      reader >> m_vNodeSataus[index];
      break;
  }
  for (size_t i = 0; i < m_vNodeSataus.size(); i++) {
    if (m_vNodeSataus[i] != static_cast<uint32_t>(ENodeStatus::READY)) {
      m_iSyncStatusMsg.data = m_vNodeSataus[i];
      return;
    }
  }
  m_iSyncStatusMsg.data = static_cast<uint32_t>(ENodeStatus::READY);
}

template <class CamDataType>
std::string HalDataSyncBase<CamDataType>::LogString(
    const can_msgs::msg::CanMsgData &msgData) {
  std::stringstream output_stream("");
  std_msgs::msg::Header head;
  head.stamp = this->now();
  output_stream << "[" << head.stamp.sec << "." << std::setw(9)
                << std::setfill('0') << head.stamp.nanosec << "], "
                << msgData.header.stamp.sec << "." << std::setw(9)
                << std::setfill('0') << msgData.header.stamp.nanosec << ", "
                << msgData.msg_name << ", " << msgData.msg_pgn << ", "
                << NS_ZF::NS_DRIVER::NS_CANBUS::Byte::Int2Hex(msgData.msg_id)
                << ", ";
  for (auto &sig : msgData.sig_datas) {
    output_stream << sig.sig_name << ", " << sig.sig_data << ", ";
  }
  return output_stream.str();
}

template <class CamDataType>
void HalDataSyncBase<CamDataType>::LogStrings(
    const can_msgs::msg::CanDatas &msgDatas) {
  // for (auto &data : msgDatas.msg_datas)
  // {
  //     LOG_FILE_SYNC() << LogString(data);
  // }
  // // LOG_FILE_SYNC() << "----end";
  // LOG_FILE_SYNC() << "";
}

END_NS_ZF_HAL_SYNC

#endif /* ZF_HAL_DATA_SYNC_BASE_H_ */
