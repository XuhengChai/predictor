#include "zf_ego_angle_front/zf_ego_angle_front_node.h"

#include <cstdio>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "chrono"
#include "math.h"
#include "zf_framework_transport/transport.h"
#include "zf_global/common/zf_global_error_code.h"
#include "zf_global/zf_global_topic_name.h"

// int main(int argc, char ** argv)
// {
//   (void) argc;
//   (void) argv;

//   printf("hello world zf_detect_tracking package\n");
//   return 0;
// }
using namespace std::chrono_literals;

BEGIN_NS_ZF_DETECTION

EgoAngleFront::EgoAngleFront(const rclcpp::NodeOptions &options, std::string name)
    : Node(name, options),
      m_fAngleDetected(0.0f),
      m_fAngleFusion(0.0f) {
  m_postfix = this->declare_parameter("cam_ns", "cam2");  // side
  m_bTrans2HMI = this->declare_parameter<bool>("trans2hmi", true);
  rclcpp::QoS qos(500);
  qos.keep_last(500);
  qos.best_effort();
  qos.durability_volatile();
  m_subVehicleInfo = this->create_subscription<VehicleInfoMsg>(
      "/vehicle_info", qos,
      std::bind(&EgoAngleFront::VehicleInfoCallback, this,
                std::placeholders::_1));

  m_pubDetectInfo =
      this->create_publisher<DetectInfoMsg>(gk_egoInfoSideCam, qos); // /ego_info_side_cam
    m_pubNodeStatus =
      this->create_publisher<NodeStateMsg>("/ego_info_side_state", qos);
  InitNodeStatusPublisher();
  if (m_bTrans2HMI) {
    InitTransmit(gk_channelDetToUi + m_postfix);
  }
  InitReceiver(gk_channelCamToDet + m_postfix);
  // std::thread thread1(&EgoAngleFront::InitReceiver, this, 0);
  // thread1.detach();
}

EgoAngleFront::~EgoAngleFront() {}

// TODO
void EgoAngleFront::CalAngle(cv::Mat &img, const uint64_t &timestamp) {
  m_bEabled = true;
  m_fAngleDetected = 12.0;
  m_fConfidence = 0.2;
}

void EgoAngleFront::DetectCbk(cv::Mat &img, uint64_t timestamp) {
  // the following is the input
  // m_img = img; // img is m_img, change img also means change img.
  int sec = timestamp / 1000000000LL;
  int nanosec = timestamp % 1000000000LL;
  float angle_fusion = m_fAngleFusion;
  // determine when to start and when to reset
  // if (m_bEabled && (m_bIsForwardDrive || !m_bIsWheelTurn)) {
  //   // Reset
  //   m_bEabled = false;
  //   m_fAngleDetected = 0.0f;
  //   m_fConfidence = 1;
  // } else if (m_bEabled || (m_bIsWheelTurn && !m_bEabled)) {
  //   this->CalAngle(img, timestamp);
  // }
  this->CalAngle(img, timestamp);
  DetectInfoMsg msgDetect;
  msgDetect.header.stamp = this->now();
  msgDetect.articulation_angle = m_fAngleDetected;
  msgDetect.confidence = m_fConfidence;
  m_pubDetectInfo->publish(msgDetect);
  // Optional for debug
  // Text confidence and angle on img
  cv::putText(m_img, "front confidence: ", cv::Point(1710, 150),
              cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(147, 20, 255), 2, 4);
  std::string sConfidence = std::to_string(m_fConfidence);
  cv::putText(m_img, sConfidence.substr(0, sConfidence.find(".") + 3),
              cv::Point(1710, 200), cv::FONT_HERSHEY_SIMPLEX, 1,
              cv::Scalar(147, 20, 255), 2, 4);
  cv::putText(m_img, "front deg: ", cv::Point(1710, 250),
              cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(147, 20, 255), 2, 4);
  std::string sAngle = std::to_string(m_fAngleDetected);
  cv::putText(m_img, sAngle.substr(0, sAngle.find(".") + 3),
              cv::Point(1710, 300), cv::FONT_HERSHEY_SIMPLEX, 1,
              cv::Scalar(147, 20, 255), 2, 4);
}

// used to start detection & update vision ego from fused ego_angle
void EgoAngleFront::VehicleInfoCallback(
    const VehicleInfoMsg::ConstSharedPtr msg) {
  // m_bIsWheelTurn = msg->is_wheel_turn;
  m_bIsWheelTurn = msg->artic_angle < 0;
  m_bIsForwardDrive = msg->is_forward_drive;
  m_fAngleFusion = msg->artic_angle * 180 / M_PI;
}

void EgoAngleFront::NodeStatusCbk() {
  if (m_iMissingCount > 0) {
    m_iMissingCount--;
  }
  if (m_iMissingCount > 10) {
    m_msgNodeStatus.node_state = static_cast<uint8_t>(ENodeStatus::ERROR);
  } else {
    m_msgNodeStatus.node_state = static_cast<uint8_t>(ENodeStatus::READY);
  }
}

void EgoAngleFront::InitNodeStatusPublisher() {
  m_msgNodeStatus.node_state = static_cast<uint8_t>(ENodeStatus::INIT);
  m_iMissingCount = 0;
  m_timerNodeSataus = create_wall_timer(100ms, [this]() {
    m_msgNodeStatus.watchdog_signal = !m_msgNodeStatus.watchdog_signal;
    m_pubNodeStatus->publish(m_msgNodeStatus);
    m_iMissingCount++;
  });
}

END_NS_ZF_DETECTION

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  // auto node = std::make_shared<MinimalDepthSubscriber>();
  const rclcpp::NodeOptions options;
  auto node = std::make_shared<NS_ZF_DETECTION::EgoAngleFront>(options,
                                                              "ego_angle_front");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
