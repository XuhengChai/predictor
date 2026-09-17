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

#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <string>

extern "C" {
#include "gst/app/gstappsink.h"
#include "gst/gst.h"
}

#include <sstream>

#include "camera_info_manager/camera_info_manager.hpp"
#include "gscam/gscam.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
// #include "zf_framework_transport/include/zf_framework_transport/transport.h"
#include "zf_framework_transport/transport.h"
#include "zf_global/common/zf_global_error_code.h"
#include "zf_global/zf_global_topic_name.h"
#include "zf_global/util/logger.h"

using namespace std::chrono_literals;

namespace gscam {

GSCam::GSCam(const rclcpp::NodeOptions &options)
    : rclcpp::Node("gscam_publisher", options),
      gsconfig_(""),
      pipeline_(NULL),
      sink_(NULL),
      camera_info_manager_(this),
      stop_signal_(false),
      m_iMissingCount(0),
      m_imgQueue(new ImgRIngBuffer(30)) {
  RCLCPP_INFO(this->get_logger(), "Hello i am node gscam_publisher.");
  // this->declare_parameter("save_dir", "/home/Pictures/camera/");
  this->declare_parameter("save_dir", "~/Pictures/camera/");
  this->get_parameter("save_dir", m_saveDir);
  m_isCompressed = this->declare_parameter("use_compressed", false);
  m_prefix = this->declare_parameter("cam_ns", "cam0");
  m_saveFormat = this->declare_parameter("format", ".jpg");
  m_bUsingRos = this->declare_parameter("use_ros", 0);
  m_iSaveFlag = this->declare_parameter("save_flag", 0);
  m_iSaveFlag = std::max(0, m_iSaveFlag);
  m_iSaveFlag = std::min(m_iSaveFlag,
                         static_cast<int>(EShowSaveFlag::FLAG_SHOW_UNDISTORT));
  m_iImgQuality = this->declare_parameter("img_write_quality", 50);
  if (m_iImgQuality < 1 || m_iImgQuality > 100) {
    m_iImgQuality = 50;
  }
  NS_ZF::LOGGER::LoggerUtil::Instance().GetDir(m_saveDir);
  m_saveDir = m_saveDir + "/" + m_prefix + '/';
  RCLCPP_INFO_STREAM(get_logger(), "--A : " << m_saveDir.c_str()
                                            << "--B : " << m_saveFormat.c_str()
                                            << "--C : " << m_isCompressed
                                            << "--D : " << m_prefix.c_str()
                                            << "--E : " << m_iSaveFlag);
  InitTransmit();
  m_lastStamp = this->get_clock()->now();
  m_dLastStamp = rclcpp::Time(m_lastStamp).seconds();

  pipeline_thread_ = std::thread([this]() { run(); });
  if (m_iSaveFlag == EShowSaveFlag::FLAG_SAVE ||
      m_iSaveFlag == EShowSaveFlag::FLAG_SAVE_UNDISTORT) {
    m_bSaveFlag = true;
    NS_ZF::LOGGER::CreateDirectory(m_saveDir);
    m_saveThread = std::thread(&GSCam::SaveImageThread, this);
    // m_saveThread.detach();
  }
  std::string pubTopicName = "/" + m_prefix + "/node_state";
  m_iCamNodeStatusMsg.node_state =
      static_cast<uint8_t>(NS_ZF::ENodeStatus::INIT);
  m_pubNodeStatus = this->create_publisher<NodeStateMsg>(pubTopicName, 50);
  m_timerNodeSataus = create_wall_timer(100ms, [this]() {
    if (m_iMissingCount > 10) {
      m_iCamNodeStatusMsg.node_state =
          static_cast<uint8_t>(NS_ZF::ENodeStatus::ERROR);
    } else {
      m_iCamNodeStatusMsg.node_state =
          static_cast<uint8_t>(NS_ZF::ENodeStatus::READY);
    }
    // NS_ZF::LOG_DEBUG() << int(m_iCamNodeStatusMsg.node_state) << ", " <<
    // m_iMissingCount;
    m_iCamNodeStatusMsg.watchdog_signal = !m_iCamNodeStatusMsg.watchdog_signal;
    m_pubNodeStatus->publish(m_iCamNodeStatusMsg);
    m_iMissingCount++;
  });
}

GSCam::~GSCam() {
  stop_signal_ = true;
  if (m_saveThread.joinable()) {
    m_saveThread.join();
  }
  pipeline_thread_.join();
}

bool GSCam::configure() {
  // Get gstreamer configuration
  // (either from environment variable or ROS param)
  bool gsconfig_rosparam_defined = false;
  char *gsconfig_env = NULL;

  const auto gsconfig_rosparam = declare_parameter("gscam_config", "");
  gsconfig_rosparam_defined = !gsconfig_rosparam.empty();
  gsconfig_env = getenv("GSCAM_CONFIG");

  if (!gsconfig_env && !gsconfig_rosparam_defined) {
    RCLCPP_FATAL(get_logger(),
                 "Problem getting GSCAM_CONFIG environment variable and "
                 "'gscam_config' rosparam is not set. This is needed to set up "
                 "a gstreamer pipeline.");
    return false;
  } else if (gsconfig_env && gsconfig_rosparam_defined) {
    RCLCPP_FATAL(get_logger(),
                 "Both GSCAM_CONFIG environment variable and 'gscam_config' "
                 "rosparam are set. "
                 "Please only define one.");
    return false;
  } else if (gsconfig_env) {
    gsconfig_ = gsconfig_env;
    RCLCPP_INFO_STREAM(get_logger(), "Using gstreamer config from env: \""
                                         << gsconfig_env << "\"");
  } else if (gsconfig_rosparam_defined) {
    gsconfig_ = gsconfig_rosparam;
    RCLCPP_INFO_STREAM(get_logger(), "Using gstreamer config from rosparam: \""
                                         << gsconfig_rosparam << "\"");
  }

  // Get additional gscam configuration
  sync_sink_ = declare_parameter("sync_sink", true);
  preroll_ = declare_parameter("preroll", false);
  use_gst_timestamps_ = declare_parameter("use_gst_timestamps", false);

  reopen_on_eof_ = declare_parameter("reopen_on_eof", false);

  // Get the camera parameters file
  camera_info_url_ = declare_parameter("camera_info_url", "");
  camera_name_ = declare_parameter("camera_name", "");

  // Get the image encoding
  image_encoding_ = declare_parameter(
      "image_encoding", std::string(sensor_msgs::image_encodings::RGB8));
  if (image_encoding_ != sensor_msgs::image_encodings::RGB8 &&
      image_encoding_ != sensor_msgs::image_encodings::MONO8 &&
      image_encoding_ != sensor_msgs::image_encodings::YUV422 &&
      image_encoding_ != "jpeg") {
    RCLCPP_FATAL_STREAM(get_logger(),
                        "Unsupported image encoding: " + image_encoding_);
  }

  camera_info_manager_.setCameraName(camera_name_);

  if (camera_info_manager_.validateURL(camera_info_url_)) {
    camera_info_manager_.loadCameraInfo(camera_info_url_);
    RCLCPP_INFO_STREAM(get_logger(),
                       "Loaded camera calibration from " << camera_info_url_);
  } else {
    RCLCPP_WARN_STREAM(
        get_logger(),
        "Camera info at: " << camera_info_url_
                           << " not found. Using an uncalibrated config.");
  }

  // Get TF Frame
  frame_id_ = declare_parameter("frame_id", "camera_frame");
  if (frame_id_ == "camera_frame") {
    RCLCPP_WARN_STREAM(get_logger(), "No camera frame_id set, using frame \""
                                         << frame_id_ << "\".");
  }

  use_sensor_data_qos_ = declare_parameter("use_sensor_data_qos", false);

  return true;
}

bool GSCam::init_stream() {
  if (!gst_is_initialized()) {
    // Initialize gstreamer pipeline
    RCLCPP_DEBUG_STREAM(get_logger(), "Initializing gstreamer...");
    gst_init(0, 0);
  }

  RCLCPP_DEBUG_STREAM(get_logger(),
                      "Gstreamer Version: " << gst_version_string());

  GError *error = 0;  // Assignment to zero is a gst requirement

  pipeline_ = gst_parse_launch(gsconfig_.c_str(), &error);
  if (pipeline_ == NULL) {
    RCLCPP_FATAL_STREAM(get_logger(), error->message);
    return false;
  }

  // Create RGB sink
  sink_ = gst_element_factory_make("appsink", NULL);
  GstCaps *caps = gst_app_sink_get_caps(GST_APP_SINK(sink_));

  // http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
  if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
    caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "RGB",
                               NULL);
  } else if (image_encoding_ == sensor_msgs::image_encodings::MONO8) {
    caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "GRAY8",
                               NULL);
  } else if (image_encoding_ == sensor_msgs::image_encodings::YUV422) {
    caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "UYVY",
                               NULL);
  } else if (image_encoding_ == "jpeg") {
    caps = gst_caps_new_simple("image/jpeg", NULL, NULL);
  }

  gst_app_sink_set_caps(GST_APP_SINK(sink_), caps);
  gst_caps_unref(caps);

  // Set whether the sink should sync
  // Sometimes setting this to true can cause a large number of frames to be
  // dropped
  gst_base_sink_set_sync(GST_BASE_SINK(sink_), (sync_sink_) ? TRUE : FALSE);

  if (GST_IS_PIPELINE(pipeline_)) {
    GstPad *outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline_), GST_PAD_SRC);
    g_assert(outpad);

    GstElement *outelement = gst_pad_get_parent_element(outpad);
    g_assert(outelement);
    gst_object_unref(outpad);

    if (!gst_bin_add(GST_BIN(pipeline_), sink_)) {
      RCLCPP_FATAL(get_logger(), "gst_bin_add() failed");
      gst_object_unref(outelement);
      gst_object_unref(pipeline_);
      return false;
    }

    if (!gst_element_link(outelement, sink_)) {
      RCLCPP_FATAL(get_logger(),
                   "GStreamer: cannot link outelement(\"%s\") -> sink\n",
                   gst_element_get_name(outelement));
      gst_object_unref(outelement);
      gst_object_unref(pipeline_);
      return false;
    }

    gst_object_unref(outelement);
  } else {
    GstElement *launchpipe = pipeline_;
    pipeline_ = gst_pipeline_new(NULL);
    g_assert(pipeline_);

    gst_object_unparent(GST_OBJECT(launchpipe));

    gst_bin_add_many(GST_BIN(pipeline_), launchpipe, sink_, NULL);

    if (!gst_element_link(launchpipe, sink_)) {
      RCLCPP_FATAL(get_logger(), "GStreamer: cannot link launchpipe -> sink");
      gst_object_unref(pipeline_);
      return false;
    }
  }

  // Calibration between ros::Time and gst timestamps
  GstClock *clock = gst_system_clock_obtain();
  GstClockTime ct = gst_clock_get_time(clock);
  gst_object_unref(clock);
  time_offset_ = now().nanoseconds() - GST_TIME_AS_NSECONDS(ct);
  RCLCPP_INFO(get_logger(), "Time offset: %.6f",
              rclcpp::Time(time_offset_).seconds());

  gst_element_set_state(pipeline_, GST_STATE_PAUSED);

  if (gst_element_get_state(pipeline_, NULL, NULL, -1) ==
      GST_STATE_CHANGE_FAILURE) {
    RCLCPP_FATAL(get_logger(),
                 "Failed to PAUSE stream, check your gstreamer configuration.");
    return false;
  } else {
    RCLCPP_DEBUG_STREAM(get_logger(), "Stream is PAUSED.");
  }

  // Create ROS camera interface
  const auto qos =
      use_sensor_data_qos_ ? rclcpp::SensorDataQoS() : rclcpp::QoS{1};
  if (image_encoding_ == "jpeg") {
    jpeg_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "camera/image_raw/compressed", qos);
    cinfo_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/camera_info", qos);
  } else {
    camera_pub_ = image_transport::create_camera_publisher(
        this, "camera/image_raw", qos.get_rmw_qos_profile());
  }

  return true;
}

void GSCam::publish_stream() {
  RCLCPP_INFO_STREAM(get_logger(), "Publishing stream...");

  // Pre-roll camera if needed
  if (preroll_) {
    RCLCPP_DEBUG(get_logger(), "Performing preroll...");

    // The PAUSE, PLAY, PAUSE, PLAY cycle is to ensure proper pre-roll
    // I am told this is needed and am erring on the side of caution.
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (gst_element_get_state(pipeline_, NULL, NULL, -1) ==
        GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to PLAY during preroll.");
      return;
    } else {
      RCLCPP_DEBUG(get_logger(), "Stream is PLAYING in preroll.");
    }

    gst_element_set_state(pipeline_, GST_STATE_PAUSED);
    if (gst_element_get_state(pipeline_, NULL, NULL, -1) ==
        GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to PAUSE.");
      return;
    } else {
      RCLCPP_INFO(get_logger(), "Stream is PAUSED in preroll.");
    }
  }

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) ==
      GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(get_logger(), "Could not start stream!");
    return;
  }
  RCLCPP_INFO(get_logger(), "Started stream.");

  // Poll the data as fast a spossible
  while (!stop_signal_ && rclcpp::ok()) {
    // This should block until a new frame is awake, this way, we'll run at the
    // actual capture framerate of the device.
    // RCLCPP_DEBUG(get_logger(), "Getting data...");
    // RCLCPP_INFO(get_logger(), "Getting data...");
    // rclcpp::Time tBegin = now();
    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
    // rclcpp::Time tEnd = now();
    // double elapsed_time_gscam = (tEnd - tBegin).seconds();
    // double fpsGscam = 1.0 / elapsed_time_gscam;
    // NS_ZF::LOG_DEBUG() << m_imgQueue->ValidSize() <<", "<< elapsed_time_gscam
    // << ", " << fpsGscam;
    if (!sample) {
      RCLCPP_ERROR(get_logger(), "Could not get gstreamer sample.");
      break;
    }
    // RCLCPP_INFO(get_logger(), "Getting GstSample...");
    GstBuffer *buf = gst_sample_get_buffer(sample);
    // RCLCPP_INFO(get_logger(), "Getting GstBuffer...");

    GstMemory *memory = gst_buffer_get_memory(buf, 0);
    // RCLCPP_INFO(get_logger(), "Getting GstMemory...");

    GstMapInfo info;

    gst_memory_map(memory, &info, GST_MAP_READ);
    // RCLCPP_INFO(get_logger(), "Getting gst_memory_map...");

    gsize &buf_size = info.size;
    guint8 *&buf_data = info.data;

    GstClockTime bt = gst_element_get_base_time(pipeline_);
    // RCLCPP_INFO(
    //   get_logger(),
    //   "New buffer: timestamp %.6f %lu %lu %.3f",
    //   GST_TIME_AS_USECONDS(buf->timestamp + bt) / 1e6 + time_offset_,
    //   buf->timestamp, bt, time_offset_);
    // RCLCPP_INFO(get_logger(), "gst_memory_map...");

#if 0
    GstFormat fmt = GST_FORMAT_TIME;
    gint64 current = -1;

    Query the current position of the stream
    if (gst_element_query_position(pipeline_, &fmt, &current)) {
      RCLCPP_INFO_STREAM(get_logger(), "Position " << current);
    }
#endif

    // Stop on end of stream
    if (!buf) {
      RCLCPP_INFO(get_logger(), "Stream ended.");
      break;
    }

    // Get the image width and height
    GstPad *pad = gst_element_get_static_pad(sink_, "sink");
    const GstCaps *caps = gst_pad_get_current_caps(pad);
    GstStructure *structure = gst_caps_get_structure(caps, 0);
    gst_structure_get_int(structure, "width", &width_);
    gst_structure_get_int(structure, "height", &height_);

    // Update header information
    sensor_msgs::msg::CameraInfo cur_cinfo =
        camera_info_manager_.getCameraInfo();
    sensor_msgs::msg::CameraInfo::SharedPtr cinfo;
    cinfo.reset(new sensor_msgs::msg::CameraInfo(cur_cinfo));
    if (use_gst_timestamps_) {
      cinfo->header.stamp =
          rclcpp::Time(GST_TIME_AS_NSECONDS(buf->pts + bt) + time_offset_);
    } else {
      cinfo->header.stamp = now();
    }
    // RCLCPP_INFO(get_logger(), "Image time stamp:
    // %.3f",cinfo->header.stamp.toSec());
    cinfo->header.frame_id = frame_id_;

    if (image_encoding_ == "jpeg") {
      sensor_msgs::msg::CompressedImage::SharedPtr img(
          new sensor_msgs::msg::CompressedImage());
      img->header = cinfo->header;
      img->format = "jpeg";
      img->data.resize(buf_size);
      std::copy(buf_data, (buf_data) + (buf_size), img->data.begin());
      jpeg_pub_->publish(*img);
      cinfo_pub_->publish(*cinfo);

      // RCLCPP_INFO(get_logger(), "publish jpeg_pub_\n");
    } else {
      if (not m_bUsingRos) {
        std::string encodedImage(reinterpret_cast<char const *>(buf_data),
                                 buf_size);
        int32_t &sec = cinfo->header.stamp.sec;
        uint32_t &nanosec = cinfo->header.stamp.nanosec;
        long long nanoseconds = sec * 1000000000LL + nanosec;
        auto send_msg = std::make_shared<NS_ZF_FRAMEWORK::MsgBase>(encodedImage,
                                                                   nanoseconds);
        m_transmitterImg->Transmit(send_msg);

        // double h_time = cinfo->header.stamp.sec +
        // cinfo->header.stamp.nanosec * 1.0e-9; double elapsed_time = h_time
        // - m_dLastStamp; double fpsHead = 1.0 / elapsed_time;
        // // NS_ZF::LOG_DEBUG() << m_imgQueue->ValidSize() << ", " <<
        // elapsed_time << ", " << fpsHead; m_dLastStamp = h_time;

        if (m_bSaveFlag) {
          std::shared_ptr<ImgData> tData = std::make_shared<ImgData>();
          tData->img = encodedImage;
          // tData->img = cv::Mat(1080, 1920, CV_8UC3, buf_data);
          tData->sec = cinfo->header.stamp.sec;
          tData->nanosec = cinfo->header.stamp.nanosec;
          m_imgQueue->push(tData);
          m_cond.notify_one();
          NodeStatusCbk();
        }
        m_bSaveFlag = !m_bSaveFlag;

        // // rclcpp::Time tEnd = now();
        // // double elapsed_time_gscam = (tEnd - tBegin).seconds();
        // // double fpsGscam = 1.0 / elapsed_time_gscam;
        // // NS_ZF::LOG_DEBUG() << m_imgQueue->ValidSize() << ", " <<
        // elapsed_time << ", " << fpsHead
        // //                    << ", " << elapsed_time_gscam << ", " <<
        // fpsGscam;
      } else {
        // Complain if the returned buffer is smaller than we expect
        const unsigned int expected_frame_size =
            width_ * height_ *
            sensor_msgs::image_encodings::numChannels(image_encoding_);

        if (buf_size < expected_frame_size) {
          RCLCPP_WARN_STREAM(
              get_logger(),
              "GStreamer image buffer underflow: Expected frame to be "
                  << expected_frame_size << " bytes but got only " << buf_size
                  << " bytes. (make sure frames are correctly encoded)");
        }

        // Construct Image message
        sensor_msgs::msg::Image::SharedPtr img(new sensor_msgs::msg::Image());

        img->header = cinfo->header;

        // Image data and metadata
        img->width = width_;
        img->height = height_;
        img->encoding = image_encoding_;
        img->is_bigendian = false;
        img->data.resize(expected_frame_size);

        // Copy only the data we received
        // Since we're publishing shared pointers, we need to copy the image so
        // we can free the buffer allocated by gstreamer
        img->step =
            width_ * sensor_msgs::image_encodings::numChannels(image_encoding_);

        std::copy(buf_data, (buf_data) + (buf_size), img->data.begin());

        // Publish the image/info
        camera_pub_.publish(img, cinfo);
        // RCLCPP_INFO(get_logger(), "publish camera_pub_\n");
      }
    }

    // Release the buffer
    if (buf) {
      gst_memory_unmap(memory, &info);
      gst_memory_unref(memory);
      gst_sample_unref(sample);
    }
  }
}

void GSCam::cleanup_stream() {
  // Clean up
  RCLCPP_INFO(get_logger(), "Stopping gstreamer pipeline...");
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = NULL;
  }
}

void GSCam::run() {
  if (!this->configure()) {
    RCLCPP_FATAL(get_logger(), "Failed to configure gscam!");
    return;
  }

  while (!stop_signal_ && rclcpp::ok()) {
    if (!this->init_stream()) {
      RCLCPP_FATAL(get_logger(), "Failed to initialize gscam stream!");
      break;
    }

    // Block while publishing
    this->publish_stream();

    this->cleanup_stream();

    RCLCPP_INFO(get_logger(), "GStreamer stream stopped!");

    if (reopen_on_eof_) {
      RCLCPP_INFO(get_logger(), "Reopening stream...");
    } else {
      RCLCPP_INFO(get_logger(), "Cleaning up stream and exiting...");
      break;
    }
  }
  rclcpp::shutdown();
}

void GSCam::SaveImageThread() {
  while (!stop_signal_) {
    {
      std::unique_lock<std::mutex> lock(m_saveMtx);
      m_cond.wait(lock, [this]() {
        if (m_imgQueue->ValidSize()) {
          return true;
        }
        return false;
      });
    }
    auto msg = m_imgQueue->pop();
    // SaveImage(msg);
    // cv_bridge::CvImageConstPtr cv_ptr =
    //     cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    // int32_t &sec = msg->sec;
    // uint32_t &nanosec = msg->nanosec;
    // long long nanoseconds = sec * 1000000000LL + nanosec;
    // auto send_msg = std::make_shared<NS_ZF_FRAMEWORK::MsgBase>(msg->img,
    // nanoseconds); m_transmitterImg->Transmit(send_msg);

    SaveImage(msg);
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::this_thread::yield();
  }
}

void GSCam::SaveImage(const std::shared_ptr<ImgData> &msg) {
  // char *raw_memory = const_cast<char *>(reinterpret_cast<char
  // const*>(msg->data.data())); cv::Mat cv_img(msg->img);
  cv::Mat cv_img(1080, 1920, CV_8UC3, const_cast<char *>(msg->img.data()));
  cv::cvtColor(cv_img, cv_img, cv::COLOR_BGR2RGB);
  std::string total_name;
  // std::string sec;
  std::ostringstream convert;

  // Extract time stamp
  convert << m_saveDir;
  if (msg->sec != 0) {
    convert << msg->sec << "_" << std::setw(9) << std::setfill('0')
            << msg->nanosec;
    // sec = std::to_string(cv_ptr->header.stamp.sec);
    // nsec = std::to_string(cv_ptr->header.stamp.nanosec);
  } else {
    m_timetmp++;
    convert << m_timetmp;
    // sec = std::to_string(m_timetmp);
  }
  convert << m_saveFormat;
  total_name = convert.str();
  // NS_ZF::LOG_DEBUG() << "total_name " << total_name;
  cv::imwrite(total_name, cv_img, {cv::IMWRITE_JPEG_QUALITY, m_iImgQuality});
}

void GSCam::InitTransmit() {
  NS_ZF_FRAMEWORK::RoleAttributes attr;
  attr.SetHostIp(NS_ZF::GlobalData::Instance().HostIp());
  attr.SetChannelName(NS_ZF::gk_channelCamToDet + m_prefix);
  NS_ZF_FRAMEWORK::Identity id;
  attr.SetId(id.HashValue());
  m_transmitterImg = NS_ZF_FRAMEWORK::Transport::Instance()
                         .CreateTransmitter<NS_ZF_FRAMEWORK::MsgBase>(
                             attr, NS_ZF_FRAMEWORK::OptionalMode::SHM);
}
void GSCam::NodeStatusCbk() {
  if (m_iMissingCount > 0) {
    m_iMissingCount--;
  }
}
// Example callbacks for appsink
// TODO(someone): enable callback-based capture
void gst_eos_cb(GstAppSink *appsink, gpointer user_data) {}
GstFlowReturn gst_new_preroll_cb(GstAppSink *appsink, gpointer user_data) {
  return GST_FLOW_OK;
}
GstFlowReturn gst_new_asample_cb(GstAppSink *appsink, gpointer user_data) {
  return GST_FLOW_OK;
}

}  // namespace gscam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gscam::GSCam)
