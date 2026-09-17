/******************************************************************************
 * Copyright 2024 ZF. All Rights Reserved.
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
 * @brief Image converter ros node, include save or show image.
 * @todo Image undistort
 * @return
 */

#ifndef ZF_DETECT_TRACKING_NODE_H
#define ZF_DETECT_TRACKING_NODE_H

#include "interface/msg/detected_ego_info.hpp"
#include "interface/msg/multi_array_with_header.hpp"
#include "interface/msg/node_state.hpp"
#include "interface/msg/vehicle_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zf_global/in/zf_detect_global.h"
#include "zf_global/in/zf_framework_global.h"
#include "zf_cam_model/zf_cam_model_truck.h"

BEGIN_NS_ZF_FRAMEWORK  // zf::framework
    class MsgBase;

template <typename M>
class Transmitter;

template <typename M>
class Receiver;

using TransmitterPtr = std::shared_ptr<Transmitter<MsgBase>>;
using ReceiverPtr = std::shared_ptr<Receiver<MsgBase>>;

END_NS_ZF_FRAMEWORK

BEGIN_NS_ZF_DETECTION

class Inference;
class BYTETracker;
class DetcetEgoAngle;

class DetcetTracking : public rclcpp::Node {
 public:
  using CamTrackData = interface::msg::MultiArrayWithHeader;
  using NodeStateMsg = interface::msg::NodeState;
  using DetectInfoMsg = interface::msg::DetectedEgoInfo;
  using VehicleInfoMsg = interface::msg::VehicleInfo;

  explicit DetcetTracking(const rclcpp::NodeOptions &,
                          std::string name = "detect_tracking");
  ~DetcetTracking();

 private:
  void VehicleInfoCallback(const VehicleInfoMsg::ConstSharedPtr msg);
  void NodeStatusCbk();
  void InitTransmit(const uint32_t &camId);
  void InitReceiver(const uint32_t &camId);
  bool IsTrackingClass(int class_id) {
    return m_setTrackClasses.count(class_id);
  }

 private:
  //   rclcpp::Node::SharedPtr m_nodeHandle;
  rclcpp::Publisher<CamTrackData>::SharedPtr m_pubTrackData;
  rclcpp::Publisher<DetectInfoMsg>::SharedPtr m_pubDetectInfo;
  rclcpp::Publisher<NodeStateMsg>::SharedPtr m_pubNodeStatus;
  rclcpp::Subscription<VehicleInfoMsg>::SharedPtr m_subVehicleInfo;
  bool m_bIsWheelTurn;
  bool m_bIsForwardDrive;
  int m_iAngle;         // which is float need /100
  int m_iMissingCount;  // which is float need /100
  rclcpp::TimerBase::SharedPtr m_timerNodeSataus;
  NodeStateMsg m_msgNodeStatus;
  NS_ZF_FRAMEWORK::TransmitterPtr m_transmitterImg[2];
  NS_ZF_FRAMEWORK::ReceiverPtr m_recvShmImg[2];
  std::set<int> m_setTrackClasses;
  std::unique_ptr<Inference> m_yoloDet;
  std::unique_ptr<BYTETracker> m_tracker[2];
  std::unique_ptr<DetcetEgoAngle> m_pDetectedAngle;
  std::shared_ptr<CameraModelTrcuk> m_pTruckCam;
  std::vector<std::string> m_postfix;
  std::string m_sSaveFolder;
  cv::Mat m_mapx;
  cv::Mat m_mapy;
  // //
  // 需要跟踪的类别，可以根据自己需求调整，筛选自己想要跟踪的对象的种类（以下对应COCO数据集类别索引）
  // std::set<int> trackClasses{0, 1, 2, 3, 5, 7}; // person, bicycle, car,
  // motorcycle, bus, truck
};

END_NS_ZF_DETECTION

#endif  // ZF_DETECT_TRACKING_NODE_H