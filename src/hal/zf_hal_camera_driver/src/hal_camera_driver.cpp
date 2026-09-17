#include "zf_hal_camera_driver/hal_camera_driver.h"

#include <stdio.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>

#include "rclcpp/serialization.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "zf_framework_transport/transport.h"
#include "zf_global/common/zf_global_error_code.h"
#include "zf_global/zf_global_topic_name.h"

// using namespace std;
BEGIN_NS_ZF

static const char OPENCV_WINDOW[] = "Image window";
using namespace std::chrono_literals;

ImageConverter::ImageConverter(const rclcpp::NodeOptions &options,
                               std::string name)
    : Node(name, options),
      m_imgQueue(new ImgRIngBuffer(30))
      //  , m_nodeHandle(std::shared_ptr<ImageConverter>(this))
      //  , m_nodeHandle(std::shared_ptr<ImageConverter>(this, [](auto *) {}))
      //  // work ,m_nodeHandle(shared_from_this()), // doesnot work since this
      //  is not completely constructed
      ,
      m_imgTrans(std::shared_ptr<ImageConverter>(this)) {
  std::cout
      << "-------------------------------Hello i am node----------------- \n";
  RCLCPP_INFO(this->get_logger(), "Hello i am node %s.", name.c_str());
  // this->declare_parameter("save_dir", "/home/Pictures/camera/");
  this->declare_parameter("save_dir", "~/Pictures/camera/");
  this->declare_parameter("format", ".jpg");
  this->get_parameter("save_dir", m_saveDir);
  this->get_parameter("format", m_saveFormat);
  m_isCompressed = this->declare_parameter("use_compressed", false);
  m_prefix = this->declare_parameter("cam_ns", "cam0");
  m_iSaveFlag = this->declare_parameter("save_flag", 0);
  m_iSaveFlag = std::max(0, m_iSaveFlag);
  m_iSaveFlag = std::min(m_iSaveFlag,
                         static_cast<int>(EShowSaveFlag::FLAG_SHOW_UNDISTORT));
  m_iImgQuality = this->declare_parameter("img_write_quality", 50);
  if (m_iImgQuality < 1 || m_iImgQuality > 100) {
    m_iImgQuality = 50;
  }
  RCLCPP_INFO_STREAM(get_logger(), "--A : " << m_saveDir.c_str()
                                            << "--B : " << m_saveFormat.c_str()
                                            << "--C : " << m_isCompressed
                                            << "--D : " << m_prefix.c_str()
                                            << "--E : " << m_iImgQuality);
  InitTransmit();

  // m_saveDir = this->get_parameter("save_dir").as_string();
  // m_saveFormat = this->get_parameter("format").as_string();
  // 创建一个订阅者订阅话题
  uint32_t quequSize = 10;
  m_lastFrame = std::chrono::steady_clock::now();
  m_lastStamp = this->get_clock()->now();
  m_lastHeadStamp = this->get_clock()->now();
  std::string subTopicName = "/" + m_prefix + "/camera/image_raw";
  std::string pubTopicName = "/" + m_prefix + "/image_converter/img";
  if (m_isCompressed) {
    // Subscribe to the compressed image topic
    subTopicName += "/compressed";
    rclcpp::QoS video_qos(10);
    video_qos.keep_last(10);
    // video_qos.best_effort();
    video_qos.reliable();
    video_qos.durability_volatile();
    m_imgSubCompressed =
        this->create_subscription<sensor_msgs::msg::CompressedImage>(
            subTopicName, video_qos,
            std::bind(&ImageConverter::ImageCompressedCallback, this,
                      std::placeholders::_1));  // 1.1M
    pubTopicName += "/compressed";
    m_imgPubCompressed =
        this->create_publisher<sensor_msgs::msg::CompressedImage>(pubTopicName,
                                                                  quequSize);
  } else {
    // m_imgSub = m_imgTrans.subscribe(subTopicName, quequSize,
    // std::bind(&ImageConverter::ImageCallback, this, std::placeholders::_1,
    // FLAG_SAVE));
    m_imgSub = m_imgTrans.subscribe(
        subTopicName, quequSize,
        std::bind(&ImageConverter::ImageCallback, this, std::placeholders::_1,
                  static_cast<EShowSaveFlag>(m_iSaveFlag)));
    m_imgPub = m_imgTrans.advertise(pubTopicName, quequSize);
  }

  // m_image_subscriber->subscribe(this, "/camera/image_raw/compressed",
  // &ImageConverter::ImageCompressedCallback, 5); auto callback =
  // std::mem_fn(&ImageConverter::ImageCompressedCallback);
  m_bIsRunning = true;
  if (m_iSaveFlag == EShowSaveFlag::FLAG_SAVE ||
      m_iSaveFlag == EShowSaveFlag::FLAG_SAVE_UNDISTORT) {
    m_saveThread = std::thread(&ImageConverter::SaveImageThread, this);
    // m_saveThread.detach();
  }
  pubTopicName = "/" + m_prefix + "/node_state";
  m_iCamNodeStatusMsg.node_state = static_cast<uint8_t>(ENodeStatus::INIT);
  m_pubNodeStatus =
      this->create_publisher<NodeStateMsg>(pubTopicName, quequSize);
  m_timerNodeSataus = create_wall_timer(100ms, [this]() {
    if (m_iMissingCount > 10) {
      m_iCamNodeStatusMsg.node_state = static_cast<uint8_t>(ENodeStatus::ERROR);
    } else {
      m_iCamNodeStatusMsg.node_state = static_cast<uint8_t>(ENodeStatus::READY);
    }
    m_iCamNodeStatusMsg.watchdog_signal = !m_iCamNodeStatusMsg.watchdog_signal;
    m_pubNodeStatus->publish(m_iCamNodeStatusMsg);
    m_iMissingCount++;
  });
  cv::namedWindow(OPENCV_WINDOW);
  // cv::startWindowThread();
}

ImageConverter::~ImageConverter() {
  m_bIsRunning = false;
  if (m_saveThread.joinable()) {
    m_saveThread.join();
  }
  cv::destroyWindow(OPENCV_WINDOW);
}

void ImageConverter::SetSaveDir(const std::string &path) { m_saveDir = path; }

void ImageConverter::SetSaveFormat(EImageFormat format) {
  switch (format) {
    case EImageFormat::JPG:
      m_saveFormat = ".jpg";
      break;
    case EImageFormat::PNG:
      m_saveFormat = ".png";
      break;
    case EImageFormat::TIFF:
      m_saveFormat = ".tiff";
      break;
    default:
      break;
  }
}

void ImageConverter::ImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg, EShowSaveFlag flag) {
  // cv_bridge::CvImageConstPtr cv_ptr;
  // try
  // {
  //   cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  //   // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  // }
  // catch (cv_bridge::Exception &e)
  // {
  //   RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  //   return;
  // }

  // m_imgPub.publish(msg);
  // std::chrono::steady_clock::time_point begin =
  // std::chrono::steady_clock::now();
  // Convert the buffer to a std::string
  /////////////////////// method1 too slow
  // std::vector<uchar> buffer;
  // cv::imencode(".jpg", cv_ptr->image, buffer, {cv::IMWRITE_JPEG_QUALITY,
  // 80}); std::string encodedImage(buffer.begin(), buffer.end());
  /////////////////////// method2 too slow
  // std::string encodedImage(msg->data.begin(), msg->data.end()); // too slow
  /////////////////////// method3
  std::string encodedImage(reinterpret_cast<char const *>(&msg->data[0]),
                           msg->data.size());
  // std::chrono::steady_clock::time_point end =
  // std::chrono::steady_clock::now(); std::cout << "Time difference = " <<
  // std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()
  // << "[µs]" << std::endl;

  // std::chrono::steady_clock::time_point begin =
  // std::chrono::steady_clock::now();
  // // Serialize the image data
  // rclcpp::Serialization<sensor_msgs::msg::Image> imgSerializer;
  // rclcpp::SerializedMessage serialized_msg;
  // imgSerializer.serialize_message(msg.get(), &serialized_msg);
  // uint8_t *serialized_data =
  // serialized_msg.get_rcl_serialized_message().buffer; size_t serialized_size
  // = serialized_msg.size();
  // // Create a std::string from the serialized data
  // // std::string encodedImage(reinterpret_cast<const char
  // *>(serialized_data), serialized_size); const auto buffer_begin =
  // serialized_msg.get_rcl_serialized_message().buffer; const auto buffer_end =
  // buffer_begin + serialized_msg.size(); std::string
  // encodedImage(buffer_begin, buffer_end);
  // std::chrono::steady_clock::time_point end =
  // std::chrono::steady_clock::now(); std::cout << "Time difference = " <<
  // std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()
  // << "[µs]" << std::endl;

  long long nanoseconds =
      msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
  // rclcpp::Time curr_stamp = this->get_clock()->now();
  // long long nanoseconds = curr_stamp.nanoseconds(); // transfer time is 2ms
  // struct timespec ts;
  // clock_gettime(CLOCK_REALTIME, &ts);
  // long long nanoseconds = ts.tv_sec * 1000000000LL + ts.tv_nsec;
  auto send_msg =
      std::make_shared<NS_ZF_FRAMEWORK::MsgBase>(encodedImage, nanoseconds);
  m_transmitterImg->Transmit(send_msg);
  NodeStatusCbk();

  // LOG_INFO() << m_prefix << msg->header.stamp.sec << "." <<
  // msg->header.stamp.nanosec;
  // switch (flag) {
  //   case FLAG_SAVE: {
  //     m_imgQueue->push(msg);
  //     // LOG_DEBUG() << m_imgQueue->ValidSize() << ", " << msg->header.stamp.sec
  //     // << "." << std::setw(9) << std::setfill('0')
  //     //             << msg->header.stamp.nanosec;
  //     // LOG_DEBUG() << m_imgQueue->ValidSize() << ", " << elapsed_time << ", "
  //     // << fpsRos;
  //     m_cond.notify_one();
  //     // cv_bridge::CvImageConstPtr cv_ptr =
  //     //     cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  //     // SaveImage(cv_ptr);
  //     break;
  //   }
  //   case FLAG_SHOW: {
  //     cv_bridge::CvImageConstPtr cv_ptr =
  //         cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  //     ShowImage(cv_ptr);
  //     // ShowFps(msg->header.stamp);
  //     break;
  //   }
  //   default:
  //     break;
  // }
  // auto origin_image_msg = cv_ptr->toImageMsg();
  // // cv::Mat resized_image;
  // // cv::resize(cv_ptr->image, resized_image, cv::Size(1280, 640), 0, 0,
  // cv::INTER_AREA);
  // // sensor_msgs::msg::Image::ConstSharedPtr resized_image_msg
  // // = cv_bridge::CvImage(cv_ptr->header, sensor_msgs::image_encodings::RGB8,
  // resized_image).toImageMsg();
  // // cv::Mat resized_image;
  // // cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(1280, 640), 0, 0,
  // cv::INTER_LINEAR);//INTER_NEAREST

  // m_imgPub.publish(origin_image_msg);
  // rclcpp::Time curr_stamp = this->get_clock()->now();
  // double elapsed_time = (curr_stamp - msg->header.stamp).seconds();
  // RCLCPP_INFO(get_logger(), "elapsed_time: %.4f", elapsed_time);
}

// void ImageConverter::ImageCompressedCallback(const
// sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg, EShowSaveFlag flag)
void ImageConverter::ImageCompressedCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
  // cv_bridge::CvImageConstPtr cv_ptr;
  // try
  // {
  //   cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  //   // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  // }

  // catch (cv_bridge::Exception &e)
  // {
  //   RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  //   return;
  // }

  auto flag = static_cast<EShowSaveFlag>(m_iSaveFlag);
  switch (flag) {
    case FLAG_SAVE: {
      cv_bridge::CvImageConstPtr cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      SaveImage(cv_ptr);
      break;
    }
    case FLAG_SHOW: {
      cv_bridge::CvImageConstPtr cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      ShowImage(cv_ptr);
      break;
    }
    default:
      break;
  }
  NodeStatusCbk();
  m_imgPubCompressed->publish(*msg);
  // auto origin_image_msg = cv_ptr->toCompressedImageMsg();

  // // cv::Mat resized_image;
  // // cv::resize(cv_ptr->image, resized_image, cv::Size(1280, 640), 0, 0,
  // cv::INTER_NEAREST);
  // // sensor_msgs::msg::CompressedImage::SharedPtr resized_image_msg
  // // = cv_bridge::CvImage(cv_ptr->header, sensor_msgs::image_encodings::RGB8,
  // resized_image).toCompressedImageMsg();

  // m_imgPubCompressed->publish(*origin_image_msg);

  // rclcpp::Time curr_stamp = this->get_clock()->now();
  // double elapsed_time = (curr_stamp - msg->header.stamp).seconds();
  // RCLCPP_INFO(get_logger(), "Compressed elapsed_time: %.4f", elapsed_time);
}
void ImageConverter::UndistortImage() {}

void ImageConverter::ShowImage(const cv_bridge::CvImageConstPtr &cv_ptr) {
  if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  // testimage.header.stamp.sec = timestamp.seconds();
  // testimage.header.stamp.nsec = timestamp.nanoseconds();
  ShowFps(cv_ptr->header.stamp);
  cv::waitKey(1);

  std::cout << "show images: at time instance:" << cv_ptr->header.stamp.sec
            << "." << cv_ptr->header.stamp.nanosec << std::endl;
}

void ImageConverter::SaveImageThread() {
  while (m_bIsRunning) {
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
    SaveImage(msg);
    // cv_bridge::CvImageConstPtr cv_ptr =
    //     cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    // SaveImage(cv_ptr);
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::this_thread::yield();
  }
}

void ImageConverter::SaveImage(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  // char *raw_memory = const_cast<char *>(reinterpret_cast<char
  // const*>(msg->data.data()));
  cv::Mat cv_img(1080, 1920, CV_8UC3, const_cast<uchar *>(&msg->data[0]));
  cv::cvtColor(cv_img, cv_img, cv::COLOR_BGR2RGB);
  std::string total_name;
  // std::string sec;
  std::ostringstream convert;

  // Extract time stamp
  convert << m_saveDir << '/';
  if (msg->header.stamp.sec != 0) {
    convert << msg->header.stamp.sec << "_" << std::setw(9) << std::setfill('0')
            << msg->header.stamp.nanosec;
    // sec = std::to_string(cv_ptr->header.stamp.sec);
    // nsec = std::to_string(cv_ptr->header.stamp.nanosec);
  } else {
    m_timetmp++;
    convert << m_timetmp;
    // sec = std::to_string(m_timetmp);
  }
  convert << m_saveFormat;
  total_name = convert.str();
  cv::imwrite(total_name, cv_img, {cv::IMWRITE_JPEG_QUALITY, m_iImgQuality});
}

void ImageConverter::SaveImage(const cv_bridge::CvImageConstPtr &cv_ptr) {
  cv::Mat cv_img = cv_ptr->image;
  std::string total_name;
  // std::string sec;
  std::ostringstream convert;

  // Extract time stamp
  convert << m_saveDir << '/';
  if (cv_ptr->header.stamp.sec != 0) {
    convert << cv_ptr->header.stamp.sec << "_" << std::setw(9)
            << std::setfill('0') << cv_ptr->header.stamp.nanosec;
    // sec = std::to_string(cv_ptr->header.stamp.sec);
    // nsec = std::to_string(cv_ptr->header.stamp.nanosec);
  } else {
    m_timetmp++;
    convert << m_timetmp;
    // sec = std::to_string(m_timetmp);
  }
  convert << m_saveFormat;
  total_name = convert.str();

  // RCLCPP_INFO(get_logger(), "------------Saved images: %s", total_name);
  // RCLCPP_INFO(get_logger(), "Saved images: %s", total_name);
  // rclcpp::Time timestamp = this->get_clock()->now();
  // testimage.header.stamp.sec = timestamp.seconds();
  // testimage.header.stamp.nsec = timestamp.nanoseconds();

  // cv::Mat grayImg;
  // cv::cvtColor(cv_img, grayImg, cv::COLOR_BGR2GRAY);
  // cv::imwrite(total_name, grayImg);
  // cv::imwrite(total_name, cv_img);

  cv::imwrite(total_name, cv_img, {cv::IMWRITE_JPEG_QUALITY, m_iImgQuality});
  // cv::imwrite(total_name, cv_img);

  // ShowFps(cv_ptr->header.stamp);

  // std::cout << "Saved images: " << total_name << " at time instance:" <<
  // timestamp.seconds() << "." << timestamp.nanoseconds() << std::endl;
  // std::cout << "Saved images: " << total_name << " at time instance:" <<
  // cv_ptr->header.stamp.sec << "." << cv_ptr->header.stamp.nanosec<<
  // std::endl;
}

void ImageConverter::ShowFps(const rclcpp::Time &headStamp) {
  rclcpp::Time curr_stamp = this->get_clock()->now();
  double elapsed_time = (curr_stamp - m_lastStamp).seconds();
  double fpsRos = 1.0 / elapsed_time;
  m_lastStamp = curr_stamp;

  elapsed_time = (headStamp - m_lastHeadStamp).seconds();
  double fpsHead = 1.0 / elapsed_time;
  m_lastHeadStamp = headStamp;

  auto now = std::chrono::steady_clock::now();
  auto ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastFrame)
          .count();
  double fps = 1000.0 / ms;
  m_lastFrame = now;
  RCLCPP_INFO(get_logger(), "FPS: %.2f --FPS head: %.2f -- %.2f", fpsRos,
              fpsHead, fps);
}

void ImageConverter::InitTransmit() {
  NS_ZF_FRAMEWORK::RoleAttributes attr;
  attr.SetHostIp(NS_ZF::GlobalData::Instance().HostIp());
  attr.SetChannelName(gk_channelCamToDet + m_prefix);
  NS_ZF_FRAMEWORK::Identity id;
  attr.SetId(id.HashValue());
  m_transmitterImg = NS_ZF_FRAMEWORK::Transport::Instance()
                         .CreateTransmitter<NS_ZF_FRAMEWORK::MsgBase>(
                             attr, NS_ZF_FRAMEWORK::OptionalMode::SHM);
}

void ImageConverter::NodeStatusCbk() {
  if (m_iMissingCount > 0) {
    m_iMissingCount--;
  }
}

END_NS_ZF

RCLCPP_COMPONENTS_REGISTER_NODE(zf::ImageConverter)

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   /*创建对应节点的共享指针对象*/
//   // auto node = std::make_shared<MinimalDepthSubscriber>();
//   auto node = std::make_shared<NS_ZF::ImageConverter>("image_convert");
//   /* 运行节点，并检测退出信号*/
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
