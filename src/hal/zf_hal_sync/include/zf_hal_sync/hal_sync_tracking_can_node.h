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
 * @brief Defines the hal datas sync ros node with tracking and can datas.
 * @return
 */

#ifndef ZF_HAL_SYNC_TRACKING_CAN_DATA_NODE_H_
#define ZF_HAL_SYNC_TRACKING_CAN_DATA_NODE_H_

#include "can_msgs/msg/radar_tracking_datas.hpp"
// #include "interface/msg/MultiArrayWithHeader.hpp"

#include "interface/msg/multi_array_with_header.hpp"
#include "interface/msg/node_state.hpp"
#include "zf_hal_sync/hal_sync_data_base.h"

//  */
BEGIN_NS_ZF_HAL_SYNC

using CamTrackingDataType = interface::msg::MultiArrayWithHeader;

class SyncTrackingCanNode : public HalDataSyncBase<CamTrackingDataType> {
 public:
  using Base = HalDataSyncBase<CamTrackingDataType>;
  using NodeStateMsg = interface::msg::NodeState;
  explicit SyncTrackingCanNode(
      const rclcpp::NodeOptions &,
      std::string name = "hal_data_tracking_sync_node");
  ~SyncTrackingCanNode();

 private:
  virtual void PubSyncData4(const can_msgs::msg::CanDatas &radar,
                            const CamTrackingDataType &trackData,
                            const can_msgs::msg::CanDatas &r2,
                            const can_msgs::msg::CanDatas &r3) override;
  virtual void PubSyncData3(const can_msgs::msg::CanDatas &radar,
                            const CamTrackingDataType &trackData,
                            const can_msgs::msg::CanDatas &r2) override;
  virtual void PubSyncData2(const can_msgs::msg::CanDatas &radar,
                            const CamTrackingDataType &trackData) override;
  virtual bool MakeDir(
      const can_msgs::msg::CanDatas &radar, const CamTrackingDataType &img,
      const can_msgs::msg::CanDatas &r2 = can_msgs::msg::CanDatas(),
      const can_msgs::msg::CanDatas &r3 = can_msgs::msg::CanDatas(),
      const CamTrackingDataType &img2 = CamTrackingDataType()) override;
  virtual void NodeStatusCallback(const std_msgs::msg::UInt32::SharedPtr msg,
                                  int index) override;

 private:
  rclcpp::Publisher<can_msgs::msg::RadarTrackingDatas>::SharedPtr m_pubSyncData;

  // rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr m_pubNodeStatus;
  // std_msgs::msg::UInt32 m_iSyncStatusMsg;
  rclcpp::Publisher<NodeStateMsg>::SharedPtr m_pubNodeStatus;
  NodeStateMsg m_iSyncNodeStatusMsg;
};

END_NS_ZF_HAL_SYNC

#endif /* ZF_HAL_SYNC_TRACKING_CAN_DATA_NODE_H_ */
