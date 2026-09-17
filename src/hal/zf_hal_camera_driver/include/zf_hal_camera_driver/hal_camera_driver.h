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
 * @brief Image converter ros node, include save or show image.
 * @todo Image undistort
 * @return
 */

#ifndef ZF_HAL_DRIVER_CAMERA_H
#define ZF_HAL_DRIVER_CAMERA_H

#include <compressed_image_transport/compressed_subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/simple_subscriber_plugin.hpp>

#include "interface/msg/node_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "zf_global/in/zf_framework_global.h"
#include "zf_global/util/ring_buffer.h"
#include "zf_global/zf_global.h"

// #include "sensor_msgs/srv/set_camera_info.hpp"

// #include "opencv2/highgui/highgui.hpp"
// #include <opencv2/imgproc.hpp>

BEGIN_NS_ZF_FRAMEWORK  // zf::framework
    class MsgBase;

template <typename M>
class Transmitter;

template <typename M>
class Receiver;

using TransmitterPtr = std::shared_ptr<Transmitter<MsgBase>>;
using ReceiverPtr = std::shared_ptr<Receiver<MsgBase>>;

END_NS_ZF_FRAMEWORK

BEGIN_NS_ZF

class ImageConverter : public rclcpp::Node {
 public:
  using ImgRIngBuffer = RingBuffer<const sensor_msgs::msg::Image>;
  using NodeStateMsg = interface::msg::NodeState;
  enum EShowSaveFlag {
    FLAG_NONE = 0,
    FLAG_SAVE = 1,
    FLAG_SHOW = 2,
    FLAG_SAVE_UNDISTORT = 3,
    FLAG_SHOW_UNDISTORT = 4
  };
  enum EImageFormat { JPG = 0, PNG = 1, TIFF = 2 };
  explicit ImageConverter(const rclcpp::NodeOptions &,
                          std::string name = "image_converter");
  ~ImageConverter();
  void SetSaveDir(const std::string &path);
  void SetSaveFormat(EImageFormat format);

 private:
  void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                     EShowSaveFlag flag);
  // void ImageCompressedCallback(const
  // sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg, EShowSaveFlag
  // flag);
  void ImageCompressedCallback(
      const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  void UndistortImage();
  void ShowImage(const cv_bridge::CvImageConstPtr &);
  void SaveImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void SaveImage(const cv_bridge::CvImageConstPtr
                     &);   //, std::string dir = "/home/Pictures/", std::string
                           //format = ".jpg"
  void SaveImageThread();  //, std::string dir = "/home/Pictures/", std::string
                           //format = ".jpg"

  void ShowFps(const rclcpp::Time &);  //, std::string dir = "/home/Pictures/",
                                       //std::string format = ".jpg"
  void InitTransmit();
  void NodeStatusCbk();

 private:
  //   rclcpp::Node::SharedPtr m_nodeHandle;
  image_transport::ImageTransport m_imgTrans;
  image_transport::Subscriber m_imgSub;
  NS_ZF_FRAMEWORK::TransmitterPtr m_transmitterImg;

  // compressed_image_transport::CompressedSubscriber m_imgSubCompressed;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
      m_imgSubCompressed;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
      m_imgPubCompressed;
  image_transport::Publisher m_imgPub;
  // EShowSaveFlag m_saveFlag = FLAG_SHOW;
  int m_timetmp = 0;
  int m_iSaveFlag = 0;
  int m_iImgQuality;
  std::string m_saveDir;
  std::string m_saveFormat;
  std::string m_prefix;
  bool m_isCompressed;
  std::chrono::steady_clock::time_point m_lastFrame;
  rclcpp::Time m_lastStamp;
  rclcpp::Time m_lastHeadStamp;
  // rclcpp::TimerBase::SharedPtr m_timer;
  // std::string m_frameId;
  bool m_bIsRunning;
  std::condition_variable m_cond;
  std::thread m_saveThread;
  std::mutex m_saveMtx;
  std::shared_ptr<ImgRIngBuffer> m_imgQueue;

  rclcpp::TimerBase::SharedPtr m_timerNodeSataus;
  rclcpp::Publisher<NodeStateMsg>::SharedPtr m_pubNodeStatus;
  NodeStateMsg m_iCamNodeStatusMsg;
  uint32_t m_iMissingCount = {};
};

END_NS_ZF

#endif  // ZF_HAL_DRIVER_CAMERA_H