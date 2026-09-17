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

#ifndef ZF_HAL_DATA_SYNC_NODE_H_
#define ZF_HAL_DATA_SYNC_NODE_H_

// #include <unordered_map>
#include <opencv2/opencv.hpp>

#include "can_msgs/msg/radar_cam_datas.hpp"
#include "zf_global/in/zf_framework_global.h"
#include "zf_hal_sync/hal_sync_data_base.h"
#include "interface/msg/node_state.hpp"

BEGIN_NS_ZF_FRAMEWORK // zf::framework
    class MsgBase;

template <typename M>
class Transmitter;

template <typename M>
class Receiver;

using TransmitterPtr = std::shared_ptr<Transmitter<MsgBase>>;
using ReceiverPtr = std::shared_ptr<Receiver<MsgBase>>;

END_NS_ZF_FRAMEWORK

// /**
//  * @namespace NS_ZF::driver::synchronization
//  */
BEGIN_NS_ZF_HAL_SYNC

// using CamDataType = sensor_msgs::msg::Image;

struct ImgShmTrans
{
  NS_ZF_FRAMEWORK::TransmitterPtr transmitterImg;
  NS_ZF_FRAMEWORK::ReceiverPtr recvShmImg;
  std::string camName;
  std::string saveDir;
  uint32_t camIdx;
};

struct CamDataTime
{
  cv::Mat img;
  std::string timestamp;
};
using CamDataType = CamDataTime;

class HalDataSyncNode : public HalDataSyncBase<CamDataType>
{
public:
  using Base = HalDataSyncBase<CamDataType>;
  using NodeStateMsg = interface::msg::NodeState;
  explicit HalDataSyncNode(const rclcpp::NodeOptions &,
                           std::string name = "hal_data_sync_node");
  ~HalDataSyncNode();
  enum ECamIndex
  {
    CAM1 = 0,
    CAM2 = 1,
    CAM1_SYNC_INDEX = 1,
    CAM2_SYNC_INDEX = 4
  };

private:
  virtual void PubSyncData5(const can_msgs::msg::CanDatas &radar,
                            const CamDataType &img,
                            const can_msgs::msg::CanDatas &r2,
                            const can_msgs::msg::CanDatas &r3,
                            const CamDataType &img2) override;
  virtual void PubSyncData4(const can_msgs::msg::CanDatas &radar,
                            const CamDataType &img,
                            const can_msgs::msg::CanDatas &r2,
                            const can_msgs::msg::CanDatas &r3) override;
  virtual void PubSyncData3(const can_msgs::msg::CanDatas &radar,
                            const CamDataType &img,
                            const can_msgs::msg::CanDatas &r2) override;
  virtual void PubSyncData2(const can_msgs::msg::CanDatas &radar,
                            const CamDataType &img) override;
  virtual bool MakeDir(
      const can_msgs::msg::CanDatas &radar, const CamDataType &img,
      const can_msgs::msg::CanDatas &r2 = can_msgs::msg::CanDatas(),
      const can_msgs::msg::CanDatas &r3 = can_msgs::msg::CanDatas(),
      const CamDataType &img2 = CamDataType()) override;
  virtual void NodeStatusCallback(const std_msgs::msg::UInt32::SharedPtr msg,
                                  int index) override;
  void InitTransmit(const uint32_t &id);
  void InitReceiver(const uint32_t &id);

private:
  //   NS_ZF_FRAMEWORK::TransmitterPtr m_transmitterImg;
  //   NS_ZF_FRAMEWORK::ReceiverPtr m_recvShmImg;
  std::unordered_map<std::uint32_t, ImgShmTrans> m_mapImgTrans;
  rclcpp::Publisher<can_msgs::msg::RadarCamDatas>::SharedPtr m_pubSyncData;
  // rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr m_pubNodeStatus;
  rclcpp::Publisher<NodeStateMsg>::SharedPtr m_pubNodeStatus;
  NodeStateMsg m_iSyncNodeStatusMsg;
  // double m_dLastTime[2] = {};
};

END_NS_ZF_HAL_SYNC

#endif /* ZF_HAL_DATA_SYNC_NODE_H_ */
