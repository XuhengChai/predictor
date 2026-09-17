/******************************************************************************
 * Copyright 2025 ZF. All Rights Reserved.
 * Author:
 *        xxx@zf.com
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
 * @brief Detect ego angle form side camera image
 * @todo Detect
 * @return
 */

#ifndef ZF_EGO_FRONT_NODE_H
#define ZF_EGO_FRONT_NODE_H

#include "interface/msg/detected_ego_info.hpp"
#include "interface/msg/node_state.hpp"
#include "interface/msg/vehicle_info.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zf_cam_model/zf_vision_trans_base.h"
#include "zf_global/in/zf_detect_global.h"
#include "zf_global/in/zf_framework_global.h"

BEGIN_NS_ZF_DETECTION

class EgoAngleFront : public VisionTransBase, public rclcpp::Node {
 public:
  using NodeStateMsg = interface::msg::NodeState;
  using DetectInfoMsg = interface::msg::DetectedEgoInfo;
  using VehicleInfoMsg = interface::msg::VehicleInfo;
  // using Base = VisionTransBase;

  explicit EgoAngleFront(const rclcpp::NodeOptions &,
                        std::string name = "ego_angle_front");
  ~EgoAngleFront();

 private:
  void CalAngle(cv::Mat &img, const uint64_t &timestamp);  // TODO implement
  void DetectCbk(cv::Mat &img, uint64_t timestamp) override;
  void VehicleInfoCallback(const VehicleInfoMsg::ConstSharedPtr msg);
  void NodeStatusCbk() override;
  void InitNodeStatusPublisher();

 private:
  // std::shared_ptr<CameraModelTrcuk> m_pTruckCam;
  rclcpp::Publisher<DetectInfoMsg>::SharedPtr m_pubDetectInfo;
  rclcpp::Publisher<NodeStateMsg>::SharedPtr m_pubNodeStatus;
  rclcpp::Subscription<VehicleInfoMsg>::SharedPtr m_subVehicleInfo;
  rclcpp::TimerBase::SharedPtr m_timerNodeSataus;
  NodeStateMsg m_msgNodeStatus;
  float m_fAngleDetected;
  float m_fConfidence;
  bool m_bIsWheelTurn = {};
  bool m_bIsForwardDrive = {};
  bool m_bEabled = {};
  float m_fAngleFusion;
  int m_iMissingCount;
};

END_NS_ZF_DETECTION

#endif  // ZF_EGO_FRONT_NODE_H