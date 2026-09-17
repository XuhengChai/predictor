// Copyright 2022 Jonathan Bohren, Clyde McQueen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GSCAM__GSCAM_HPP_
#define GSCAM__GSCAM_HPP_

#include <stdexcept>
#include <string>

extern "C"
{
#include "gst/gst.h"
#include "gst/app/gstappsink.h"
}

#include "rclcpp/rclcpp.hpp"

#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"

#include <opencv2/opencv.hpp>
#include "zf_global/in/zf_framework_global.h"
#include "zf_global/util/ring_buffer.h"
#include "interface/msg/node_state.hpp"

// #include "sensor_msgs/srv/set_camera_info.hpp"

// #include "opencv2/highgui/highgui.hpp"
// #include <opencv2/imgproc.hpp>

BEGIN_NS_ZF_FRAMEWORK // zf::framework
    class MsgBase;

template <typename M>
class Transmitter;

template <typename M>
class Receiver;

using TransmitterPtr = std::shared_ptr<Transmitter<MsgBase>>;
using ReceiverPtr = std::shared_ptr<Receiver<MsgBase>>;

END_NS_ZF_FRAMEWORK

namespace gscam
{
  struct ImgData
  {
    // cv::Mat img;
    std::string img;
    int32_t sec = {};
    uint32_t nanosec = {};
  };

  class GSCam : public rclcpp::Node
  {
  public:
    using ImgRIngBuffer = NS_ZF::RingBuffer<ImgData>;
    using NodeStateMsg = interface::msg::NodeState;
    enum EShowSaveFlag
    {
      FLAG_NONE = 0,
      FLAG_SAVE = 1,
      FLAG_SHOW = 2,
      FLAG_SAVE_UNDISTORT = 3,
      FLAG_SHOW_UNDISTORT = 4
    };
    explicit GSCam(const rclcpp::NodeOptions &options);
    ~GSCam();

  private:
    bool configure();
    bool init_stream();
    void publish_stream();
    void cleanup_stream();

    void run();
    void InitTransmit();
    void NodeStatusCbk();
    void SaveImage(const std::shared_ptr<ImgData> &); //, std::string dir = "/home/Pictures/", std::string format = ".jpg"
    void SaveImageThread();

    // General gstreamer configuration
    std::string gsconfig_;

    // Gstreamer structures
    GstElement *pipeline_;
    GstElement *sink_;

    // Appsink configuration
    bool sync_sink_;
    bool preroll_;
    bool reopen_on_eof_;
    bool use_gst_timestamps_;

    // Camera publisher configuration
    std::string frame_id_;
    int width_, height_;
    std::string image_encoding_;
    std::string camera_name_;
    std::string camera_info_url_;
    bool use_sensor_data_qos_;

    // ROS Inteface
    // Calibration between ros::Time and gst timestamps
    uint64_t time_offset_;
    camera_info_manager::CameraInfoManager camera_info_manager_;
    image_transport::CameraPublisher camera_pub_;
    // Case of a jpeg only publisher
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr jpeg_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cinfo_pub_;

    // Poll gstreamer on a separate thread
    std::thread pipeline_thread_;
    std::atomic<bool> stop_signal_;

  private:
    NS_ZF_FRAMEWORK::TransmitterPtr m_transmitterImg;
    int m_timetmp = 0;
    int m_iSaveFlag = 0;
    bool m_bSaveFlag = {};
    bool m_bUsingRos = {};
    int m_iImgQuality;
    std::string m_saveDir;
    std::string m_saveFormat;
    std::string m_prefix;
    bool m_isCompressed;
    rclcpp::Time m_lastStamp;
    double m_dLastStamp;
    // rclcpp::TimerBase::SharedPtr m_timer;
    // std::string m_frameId;
    std::condition_variable m_cond;
    std::thread m_saveThread;
    std::mutex m_saveMtx;
    std::shared_ptr<ImgRIngBuffer> m_imgQueue;

    rclcpp::TimerBase::SharedPtr m_timerNodeSataus;
    rclcpp::Publisher<NodeStateMsg>::SharedPtr m_pubNodeStatus;
    NodeStateMsg m_iCamNodeStatusMsg;
    uint32_t m_iMissingCount;
  };

} // namespace gscam

#endif // GSCAM__GSCAM_HPP_
