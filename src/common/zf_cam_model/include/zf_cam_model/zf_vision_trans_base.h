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

#ifndef ZF_VISION_TRANS_BASE_NODE_H // #############
#define ZF_VISION_TRANS_BASE_NODE_H

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zf_cam_model/zf_cam_model_truck.h"
#include "zf_global/in/zf_detect_global.h"
#include "zf_global/in/zf_framework_global.h"

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

class VisionTransBase {
 public:
  VisionTransBase();
  ~VisionTransBase();

 public:
  virtual void DetectCbk(cv::Mat &img, uint64_t timestamp) {};
  virtual void NodeStatusCbk() {};

 protected:
  void InitTransmit(const std::string &channel_name);
  void InitReceiver(const std::string &channel_name);
  void SetSavePath(const std::string &path) {
    m_bSaveImg = true;
    m_strSavePath = path;
  };
  void SetTrans2HMI(bool val) { m_bTrans2HMI = val; };

 protected:
  //   rclcpp::Node::SharedPtr m_nodeHandle;
  std::shared_ptr<CameraModelTrcuk> m_pTruckCam;
  cv::Mat m_img;
  std::string m_postfix;
  bool m_bTrans2HMI = {};

 private:
  bool m_bSaveImg = {};
  std::string m_strSavePath;
  NS_ZF_FRAMEWORK::TransmitterPtr m_transmitterImg;
  NS_ZF_FRAMEWORK::ReceiverPtr m_recvShmImg;
};

END_NS_ZF_DETECTION

#endif  // ZF_VISION_TRANS_BASE_NODE_H