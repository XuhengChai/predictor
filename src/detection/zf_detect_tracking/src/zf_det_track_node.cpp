#include "zf_detect_tracking/zf_det_track_node.h"

#include <cstdio>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
// #include <opencv2/cudawarping.hpp>

#include "chrono"
#include "math.h"
#include "zf_detect_tracking/tracking/zf_byte_tracker.h"
#include "zf_detect_tracking/zf_det_ego_angle.h"
#include "zf_detect_tracking/zf_infer_tensorRT.h"
#include "zf_framework_transport/transport.h"
#include "zf_global/common/zf_global_error_code.h"
#include "zf_global/zf_global_topic_name.h"
#include "zf_global/util/logger.h"

// int main(int argc, char ** argv)
// {
//   (void) argc;
//   (void) argv;

//   printf("hello world zf_detect_tracking package\n");
//   return 0;
// }
using namespace std::chrono_literals;

BEGIN_NS_ZF_DETECTION

DetcetTracking::DetcetTracking(const rclcpp::NodeOptions &options,
                               std::string name)
    : Node(name, options),
      m_yoloDet(new Infer_YOLOv8()),
      // m_tracker(new BYTETracker(30, 30)),
      m_pDetectedAngle(new DetcetEgoAngle),
      m_pTruckCam(new CameraModelTrcuk) {
  m_setTrackClasses = {0, 1, 2, 3, 5, 7};
  m_postfix.push_back("cam2");  // left
  m_postfix.push_back("cam1");  // right
  m_tracker[0] = std::make_unique<BYTETracker>(30, 30);
  m_tracker[1] = std::make_unique<BYTETracker>(30, 30);
  const std::string engine_file_path = "/home/nvidia/params/yolov8m.engine";
  LOG_DEBUG() << "Init begin, waiting: " << engine_file_path;
  m_yoloDet->Init(engine_file_path);
  LOG_DEBUG() << "Init done: " << engine_file_path;
  rclcpp::QoS qos(500);
  qos.keep_last(500);
  qos.best_effort();
  qos.durability_volatile();
  std::string subTopicName = "";
  std::string pubTopicName = "";
  NS_ZF::LOGGER::LoggerUtil::Instance().GetDir(m_sSaveFolder);
  m_subVehicleInfo = this->create_subscription<VehicleInfoMsg>(
      "/vehicle_info", 100,
      std::bind(&DetcetTracking::VehicleInfoCallback, this,
                std::placeholders::_1));
  m_pubTrackData =
      this->create_publisher<CamTrackData>("/tracked_objects", qos);
  m_pubNodeStatus =
      this->create_publisher<NodeStateMsg>("/perception_state", qos);
  m_pubDetectInfo =
      this->create_publisher<DetectInfoMsg>("/detected_info", qos);
  m_msgNodeStatus.node_state = static_cast<uint8_t>(ENodeStatus::INIT);
  m_iMissingCount = 0;
  m_timerNodeSataus = create_wall_timer(100ms, [this]() {
    m_msgNodeStatus.watchdog_signal = !m_msgNodeStatus.watchdog_signal;
    m_pubNodeStatus->publish(m_msgNodeStatus);
    m_iMissingCount++;
  });
  // m_pTruckCam->GetMapXYPerspective(m_mapx, m_mapy);
  m_pTruckCam->GetMapXYPolyconic(m_mapx, m_mapy);
  // m_pTruckCam->GetMapXYBev(m_mapx, m_mapy);
  m_yoloDet->SetMapXY(m_mapx, m_mapy);  // using undistort here

  for (int i = 0; i < 2; i++) {
    InitTransmit(i);
    InitReceiver(i);
  }
  // std::thread thread1(&DetcetTracking::InitReceiver, this, 0);
  // thread1.detach();
}

DetcetTracking::~DetcetTracking() {}

void DetcetTracking::InitTransmit(const uint32_t &camId) {
  NS_ZF_FRAMEWORK::RoleAttributes attr;
  attr.SetHostIp(NS_ZF::GlobalData::Instance().HostIp());
  attr.SetChannelName(gk_channelDetToUi + m_postfix[camId]);
  NS_ZF_FRAMEWORK::Identity id;
  attr.SetId(id.HashValue());
  m_transmitterImg[camId] = NS_ZF_FRAMEWORK::Transport::Instance()
                                .CreateTransmitter<NS_ZF_FRAMEWORK::MsgBase>(
                                    attr, NS_ZF_FRAMEWORK::OptionalMode::SHM);
}

void DetcetTracking::VehicleInfoCallback(
    const VehicleInfoMsg::ConstSharedPtr msg) {
  // m_bIsWheelTurn = msg->is_wheel_turn;
  m_bIsWheelTurn = msg->artic_angle < 0;
  m_bIsForwardDrive = msg->is_forward_drive;
  m_iAngle = static_cast<int>(msg->artic_angle * 18000 / M_PI);
}

void DetcetTracking::NodeStatusCbk() {
  if (m_iMissingCount > 0) {
    m_iMissingCount--;
  }
  if (m_iMissingCount > 10) {
    m_msgNodeStatus.node_state = static_cast<uint8_t>(ENodeStatus::ERROR);
  } else {
    m_msgNodeStatus.node_state = static_cast<uint8_t>(ENodeStatus::READY);
  }
}

void DetcetTracking::InitReceiver(const uint32_t &camId) {
  NS_ZF_FRAMEWORK::RoleAttributes attr;
  attr.SetHostIp(NS_ZF::GlobalData::Instance().HostIp());
  attr.SetChannelName(gk_channelCamToDet + m_postfix[camId]);
  NS_ZF_FRAMEWORK::Identity id;
  attr.SetId(id.HashValue());

  auto listener = [this, camId](
                      const std::shared_ptr<NS_ZF_FRAMEWORK::MsgBase> &msg,
                      const NS_ZF_FRAMEWORK::MessageInfo &msg_info,
                      const NS_ZF_FRAMEWORK::RoleAttributes &attr) {
    (void)msg_info;
    bool isRightCam = (camId == 1);
    // TODO use thread to handle

    // struct timespec ts;
    // clock_gettime(CLOCK_REALTIME, &ts);
    // LOG_INFO() << camId << "--thread ID--: " << std::this_thread::get_id();
    // LOG_INFO() << "-------I heared: " << msg->Data().size()
    //            << " DELAY: " << (nanoseconds - msg->TimeStamp()) / 1000LL
    //            << " send tmp: " << msg->TimeStamp() / 1000000000LL
    //            << "." << msg->TimeStamp() % 1000000000LL
    //            //  << " recv tmp: " << ts.tv_sec << "." << ts.tv_nsec;
    //            << " recv tmp: " << nanoseconds / 1000000000LL
    //            << "." << nanoseconds % 1000000000LL;
    // Decode data into Mat
    // std::chrono::steady_clock::time_point begin =
    // std::chrono::steady_clock::now();
    auto start = std::chrono::system_clock::now();

    char *raw_memory = (const_cast<char *>(msg->Data().data()));
    // std::vector<uint8_t> buffer(raw_memory, raw_memory + msg->Data().size());
    cv::Mat decodedImage(1080, 1920, CV_8UC3, raw_memory);
    cv::cvtColor(decodedImage, decodedImage, cv::COLOR_BGR2RGB);

    // cv::Mat res;
    // cv::Size size = cv::Size{640, 640};

    if (isRightCam) {
      // // auto un_start = std::chrono::system_clock::now();
      // cv::imwrite("/home/nvidia/Pictures/camera/1.jpg", decodedImage, {cv::IMWRITE_JPEG_QUALITY, 50});
      // cv::Mat undistortImage(decodedImage.size(), decodedImage.type());
      // cv::remap(decodedImage, undistortImage, m_mapx, m_mapy, cv::INTER_LINEAR);
      // // auto un_end = std::chrono::system_clock::now();
      // // auto un_tc = std::chrono::duration_cast<std::chrono::microseconds>(
      // //                  un_end - un_start)
      // //                  .count() /
      // //              1000;
      // cv::imwrite("/home/nvidia/Pictures/camera/2.jpg", undistortImage, {cv::IMWRITE_JPEG_QUALITY, 50});

      // decodedImage = undistortImage;
      
      bool flag_undistort = m_yoloDet->GetUndistortFlag();
      std::vector<NS_ZF_DETECTION::DetectObject> objs;
      m_yoloDet->Infer(decodedImage, objs);
      std::vector<NS_ZF_DETECTION::DetectObject> objects;
      for (long unsigned int j = 0; j < objs.size(); j++) {
        if (this->IsTrackingClass(objs[j].label)) {
          objects.push_back(objs[j]);
        }
      }
      std::vector<NS_ZF_DETECTION::STrack> output_stracks =
          m_tracker[camId]->update(objects);
      // if (m_pDetectedAngle->Enabled() && (m_bIsForwardDrive ||
      // !m_bIsWheelTurn)) {
      //   m_pDetectedAngle->Reset();
      // } else if (m_pDetectedAngle->Enabled() ||
      //            (m_bIsWheelTurn && !m_pDetectedAngle->Enabled())) {
      //   float angle = m_iAngle / 100.0;
      //   m_pDetectedAngle->CalAngle(decodedImage, angle);
      // }

      auto end = std::chrono::system_clock::now();
      auto tc =
          std::chrono::duration_cast<std::chrono::microseconds>(end - start)
              .count() /
          1000;

      CamTrackData trackData;
      trackData.header.stamp = this->now();
      trackData.time_stamp = msg->TimeStamp()/1000000000.0;
      trackData.save_path = m_sSaveFolder;
      //[123.956,585.926,362.745,830.465,2,0.731,2,0,]
      //(x, y, x, y, id, conf, cls, ind)
      std_msgs::msg::MultiArrayDimension dimRow;
      dimRow.label = "rows";
      dimRow.size = output_stracks.size();
      dimRow.stride = output_stracks.size() * 8;
      trackData.array.layout.dim.push_back(dimRow);
      std_msgs::msg::MultiArrayDimension dimCol;
      dimCol.label = "cols";
      dimCol.size = 8;
      dimCol.stride = 8;
      trackData.array.layout.dim.push_back(dimCol);
      for (int i = 0; i < output_stracks.size(); i++) {
        std::vector<float> tlwh = output_stracks[i].tlwh;
        // bool vertical = tlwh[2] / tlwh[3] > 1.6;
        // if (tlwh[2] * tlwh[3] > 20 && !vertical)
        std::vector<float> data = output_stracks[i].to_xyxyIdConfCls();
        data.push_back(i);
        trackData.array.data.insert(trackData.array.data.end(), data.begin(),
                                    data.end());

        if (tlwh[2] * tlwh[3] > 20) {
          cv::Scalar s =
              m_tracker[camId]->get_color(output_stracks[i].track_id);
          if (flag_undistort) {
            auto tl = m_pTruckCam->UndistortPixel2Origin(tlwh[0], tlwh[1]);
            cv::putText(decodedImage,
                        cv::format("%d", output_stracks[i].track_id),
                        cv::Point(tl.x, tl.y - 5), 0, 0.6,
                        cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
            std::vector<cv::Point> rect = {
                tl,
                m_pTruckCam->UndistortPixel2Origin(tlwh[0] + tlwh[2], tlwh[1]),
                m_pTruckCam->UndistortPixel2Origin(tlwh[0] + tlwh[2],
                                                   tlwh[1] + tlwh[3]),
                m_pTruckCam->UndistortPixel2Origin(tlwh[0], tlwh[1] + tlwh[3]),
            };
            // Draw lines and polygons
            cv::polylines(decodedImage, rect, true, s, 2);
          } else {
            cv::putText(decodedImage,
                        cv::format("%d", output_stracks[i].track_id),
                        cv::Point(tlwh[0], tlwh[1] - 5), 0, 0.6,
                        cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
            cv::rectangle(decodedImage,
                          cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
          }
        }
      }
      m_pubTrackData->publish(trackData);
      // DetectInfoMsg msgDetect;
      // msgDetect.header.stamp = this->now();
      // auto propEgo = m_pDetectedAngle->GetEgoProp();
      // msgDetect.articulation_angle = propEgo.angle;
      // msgDetect.confidence = propEgo.confidence;
      // msgDetect.last_pixel_u = propEgo.u;
      // msgDetect.last_pixel_v = propEgo.v;
      // m_pubDetectInfo->publish(msgDetect);

      // for (size_t i = 0; i < propEgo.u.size(); i++) {
      //   cv::Point p(propEgo.u.at(i), propEgo.v.at(i));
      //   cv::circle(decodedImage, p, 0, cv::Scalar(147, 20, 255), 5);
      // }

      // cv::putText(decodedImage, "confidence: ", cv::Point(1710, 150),
      //             cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(147, 20, 255), 2,
      //             4);
      // std::string sConfidence = std::to_string(propEgo.confidence);
      // cv::putText(decodedImage,
      //             sConfidence.substr(0, sConfidence.find(".") + 3),
      //             cv::Point(1710, 200), cv::FONT_HERSHEY_SIMPLEX, 1,
      //             cv::Scalar(147, 20, 255), 2, 4);
      // cv::putText(decodedImage, "Detected deg: ", cv::Point(1710, 250),
      //             cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(147, 20, 255), 2,
      //             4);
      // std::string sAngle = std::to_string(propEgo.angle);
      // cv::putText(decodedImage, sAngle.substr(0, sAngle.find(".") + 3),
      //             cv::Point(1710, 300), cv::FONT_HERSHEY_SIMPLEX, 1,
      //             cv::Scalar(147, 20, 255), 2, 4);

      cv::putText(decodedImage,
                  cv::format("tc: %ld fps: %d num: %ld", tc, int(1000 / tc),
                             output_stracks.size()),
                  cv::Point(0, 30), 0, 0.6, cv::Scalar(0, 0, 255), 2,
                  cv::LINE_AA);
    }

    // // cv::putText(img, cv::format("frame: %d fps: %d num: %ld", num_frames,
    // // num_frames * 1000000 / total_ms, output_stracks.size()),
    // cv::putText(decodedImage,
    //             cv::format("tc: %d fps: %d num: %ld", tc, int(1000 / tc),
    //                        output_stracks.size()),
    //             cv::Point(0, 30), 0, 0.6, cv::Scalar(0, 0, 255), 2,
    //             cv::LINE_AA);
    std::vector<cv::String> textLines;
    rclcpp::Time curr_stamp = this->get_clock()->now();
    long long nanoseconds = curr_stamp.nanoseconds();
    textLines.push_back(cv::format("send: %lld.%lld",
                                   msg->TimeStamp() / 1000000000LL,
                                   msg->TimeStamp() % 1000000000LL));
    textLines.push_back(cv::format("recv: %lld.%lld",
                                   nanoseconds / 1000000000LL,
                                   nanoseconds % 1000000000LL));
    textLines.push_back(
        cv::format("delay: %lld", (nanoseconds - msg->TimeStamp()) / 1000LL));
    textLines.push_back(cv::format("from: %s", m_postfix[camId].c_str()));
    cv::Point pCv(200, 330);
    float fontScale = 0.6;
    int thickness = 1;
    int fontFace = cv::LINE_AA;
    int baseline = 0;
    for (size_t i = 0; i < textLines.size(); i++) {
      std::string text = textLines[i];
      if (!text.empty()) {
        cv::Size textSize =
            cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
        cv::Point drawPoint(pCv.x,
                            pCv.y + (textSize.height + 10 + baseline) * i);
        cv::putText(decodedImage, text, drawPoint, cv::FONT_HERSHEY_DUPLEX,
                    fontScale, cv::Scalar(0, 0, 255), thickness, fontFace);
      }
    }

    // cv::cvtColor(decodedImage, decodedImage, cv::COLOR_RGB2GRAY);
    char *decodedImage_memory = (reinterpret_cast<char *>(decodedImage.data));
    std::string encodedImage(decodedImage_memory, msg->Data().size());
    auto send_msg = std::make_shared<NS_ZF_FRAMEWORK::MsgBase>(
        encodedImage, msg->TimeStamp());  // int(m_postfix[camId].back())
    // msg->ParseFromString(encodedImage);
    m_transmitterImg[camId]->Transmit(send_msg);
    NodeStatusCbk();

    // rclcpp::Time curr_stamp = this->get_clock()->now();
    // long long nanoseconds = curr_stamp.nanoseconds();
    // std::cout
    //     // << "Time imdecode = " <<
    //     // std::chrono::duration_cast<std::chrono::microseconds>(end -
    //     // begin).count() << "[µs]"
    //     // << "I heared: " << buffer.size()
    //     << " DELAY: " << (nanoseconds - msg->TimeStamp()) / 1000LL
    //     << " send tmp: " << msg->TimeStamp() / 1000000000LL << "."
    //     << msg->TimeStamp() % 1000000000LL
    //     << " recv tmp: " << nanoseconds / 1000000000LL << "."
    //     << nanoseconds % 1000000000LL << std::endl;

    // std::string total_name;
    // std::ostringstream convert;
    // // Extract time stamp
    // convert << "/home/nvidia/Pictures/camera/";
    // convert << msg->TimeStamp() / 1000000000LL
    //         << "_" << msg->TimeStamp() % 1000000000LL;
    // convert << ".jpg";
    // total_name = convert.str();
    // // LOG_INFO() << "-------I heared imwrite: "  << total_name.c_str();
    // cv::imwrite(total_name, decodedImage, {cv::IMWRITE_JPEG_QUALITY, 50});
  };

  m_recvShmImg[camId] =
      NS_ZF_FRAMEWORK::Transport::Instance()
          .CreateReceiver<NS_ZF_FRAMEWORK::MsgBase>(
              attr, listener, NS_ZF_FRAMEWORK::OptionalMode::SHM);
}

END_NS_ZF_DETECTION

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  // auto node = std::make_shared<MinimalDepthSubscriber>();
  const rclcpp::NodeOptions options;
  auto node = std::make_shared<NS_ZF_DETECTION::DetcetTracking>(
      options, "detect_tracking");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// const std::vector<std::string> CLASS_NAMES = {
//     "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
//     "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking
//     meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
//     "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
//     "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports
//     ball", "kite", "baseball bat", "baseball glove", "skateboard",
//     "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
//     "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
//     "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
//     "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
//     "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
//     "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
//     "teddy bear", "hair drier", "toothbrush"};

// const std::vector<std::vector<unsigned int>> COLORS = {
//     {0, 114, 189}, {217, 83, 25}, {237, 177, 32}, {126, 47, 142}, {119, 172,
//     48}, {77, 190, 238}, {162, 20, 47}, {76, 76, 76}, {153, 153, 153}, {255,
//     0, 0}, {255, 128, 0}, {191, 191, 0}, {0, 255, 0}, {0, 0, 255}, {170, 0,
//     255}, {85, 85, 0}, {85, 170, 0}, {85, 255, 0}, {170, 85, 0}, {170, 170,
//     0}, {170, 255, 0}, {255, 85, 0}, {255, 170, 0}, {255, 255, 0}, {0, 85,
//     128}, {0, 170, 128}, {0, 255, 128}, {85, 0, 128}, {85, 85, 128}, {85,
//     170, 128}, {85, 255, 128}, {170, 0, 128}, {170, 85, 128}, {170, 170,
//     128}, {170, 255, 128}, {255, 0, 128}, {255, 85, 128}, {255, 170, 128},
//     {255, 255, 128}, {0, 85, 255}, {0, 170, 255}, {0, 255, 255}, {85, 0,
//     255}, {85, 85, 255}, {85, 170, 255}, {85, 255, 255}, {170, 0, 255}, {170,
//     85, 255}, {170, 170, 255}, {170, 255, 255}, {255, 0, 255}, {255, 85,
//     255}, {255, 170, 255}, {85, 0, 0}, {128, 0, 0}, {170, 0, 0}, {212, 0, 0},
//     {255, 0, 0}, {0, 43, 0}, {0, 85, 0}, {0, 128, 0}, {0, 170, 0}, {0, 212,
//     0}, {0, 255, 0}, {0, 0, 43}, {0, 0, 85}, {0, 0, 128}, {0, 0, 170}, {0, 0,
//     212}, {0, 0, 255}, {0, 0, 0}, {36, 36, 36}, {73, 73, 73}, {109, 109,
//     109}, {146, 146, 146}, {182, 182, 182}, {219, 219, 219}, {0, 114, 189},
//     {80, 183, 189}, {128, 128, 0}};

// //
// 需要跟踪的类别，可以根据自己需求调整，筛选自己想要跟踪的对象的种类（以下对应COCO数据集类别索引）
// std::set<int> trackClasses{0, 1, 2, 3, 5, 7}; // person, bicycle, car,
// motorcycle, bus, truck

// bool isTrackingClass(int class_id)
// {
//     return trackClasses.count(class_id);
// }

// void draw_objects(const cv::Mat &image,
//                   cv::Mat &res,
//                   const std::vector<DetectObject> &objs,
//                   const std::vector<std::string> &CLASS_NAMES,
//                   const std::vector<std::vector<unsigned int>> &COLORS)
// {
//     res = image.clone();
//     for (auto &obj : objs)
//     {
//         cv::Scalar color = cv::Scalar(COLORS[obj.label][0],
//         COLORS[obj.label][1], COLORS[obj.label][2]); cv::rectangle(res,
//         cv::Rect(obj.x, obj.y, obj.width, obj.height), color, 2);

//         char text[256];
//         sprintf(text, "%s %.1f%%", CLASS_NAMES[obj.label].c_str(), obj.prob *
//         100);

//         int baseLine = 0;
//         cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX,
//         0.4, 1, &baseLine);

//         int x = (int)obj.x;
//         int y = (int)obj.y + 1;

//         if (y > res.rows)
//             y = res.rows;

//         cv::rectangle(res, cv::Rect(x, y, label_size.width, label_size.height
//         + baseLine), {0, 0, 255}, -1);

//         cv::putText(res, text, cv::Point(x, y + label_size.height),
//         cv::FONT_HERSHEY_SIMPLEX, 0.4, {255, 255, 255}, 1);
//     }
// }

// END_NS_ZF_DETECTION

// int main(int argc, char **argv)
// {
//     cudaSetDevice(0);
//     const std::string engine_file_path =
//     "/home/nvidia/Documents/dataC7_08/yolov8m.engine"; const std::string path
//     = "/home/nvidia/Documents/dataC7_08/20240312/1501/cam/";
//     std::vector<std::string> imagePathList;
//     bool isVideo{false};
//     auto yolov8 = new NS_ZF_DETECTION::Infer_YOLOv8();
//     yolov8->Init(engine_file_path);

//     if (NS_ZF_DETECTION::IsFile(path))
//     {
//         std::string suffix = path.substr(path.find_last_of('.') + 1);
//         if (suffix == "jpg" || suffix == "jpeg" || suffix == "png")
//         {
//             imagePathList.push_back(path);
//         }
//         else
//         {
//             printf("suffix %s is wrong !!!\n", suffix.c_str());
//             std::abort();
//         }
//     }
//     else if (NS_ZF_DETECTION::IsFolder(path))
//     {
//         cv::glob(path + "/*.jpg", imagePathList);
//     }
//     else
//     {
//         std::cout << "Invalid path: " << std::endl;
//         return 0;
//     }

//     cv::Mat res, image;
//     cv::Size size = cv::Size{640, 640};
//     std::vector<NS_ZF_DETECTION::DetectObject> objs;

//     // cv::namedWindow("result", cv::WINDOW_AUTOSIZE);
//     NS_ZF_DETECTION::BYTETracker tracker(16, 16);

//     int num_frames = 0;
//     int total_ms = 0;
//     for (auto &path : imagePathList)
//     {
//         objs.clear();
//         image = cv::imread(path);
//         num_frames++;

//         // std::cout << "read path is: " << path.c_str() << std::endl;
//         auto start = std::chrono::system_clock::now();
//         yolov8->Infer(image, objs);
//         // auto end = std::chrono::system_clock::now();

//         // yolo output format to bytetrack input format, and filter bbox by
//         class id std::vector<NS_ZF_DETECTION::DetectObject> objects; for
//         (long unsigned int j = 0; j < objs.size(); j++)
//         {
//             if (NS_ZF_DETECTION::isTrackingClass(objs[j].label))
//             {
//                 objects.push_back(objs[j]);
//             }
//         }

//         // track
//         std::vector<NS_ZF_DETECTION::STrack> output_stracks =
//         tracker.update(objects);

//         auto end = std::chrono::system_clock::now();
//         auto tc = std::chrono::duration_cast<std::chrono::microseconds>(end -
//         start).count() / 1000;

//         for (int i = 0; i < output_stracks.size(); i++)
//         {
//             std::vector<float> tlwh = output_stracks[i].tlwh;
//             // bool vertical = tlwh[2] / tlwh[3] > 1.6;
//             // if (tlwh[2] * tlwh[3] > 20 && !vertical)
//             if (tlwh[2] * tlwh[3] > 20)
//             {
//                 cv::Scalar s = tracker.get_color(output_stracks[i].track_id);
//                 cv::putText(image, cv::format("%d",
//                 output_stracks[i].track_id), cv::Point(tlwh[0], tlwh[1] - 5),
//                             0, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
//                 cv::rectangle(image, cv::Rect(tlwh[0], tlwh[1], tlwh[2],
//                 tlwh[3]), s, 2);
//             }
//         }
//         // cv::putText(img, cv::format("frame: %d fps: %d num: %ld",
//         num_frames, num_frames * 1000000 / total_ms, output_stracks.size()),
//         cv::putText(image, cv::format("tc: %d fps: %d num: %ld", tc, int(1000
//         / tc), output_stracks.size()),
//                     cv::Point(0, 30), 0, 0.6, cv::Scalar(0, 0, 255), 2,
//                     cv::LINE_AA);

//         // std::cout << "objs  is: " << objs.size() << std::endl;
//         // draw_objects(image, res, objs, NS_ZF_DETECTION::CLASS_NAMES,
//         NS_ZF_DETECTION::COLORS);
//         // auto tc =
//         (double)std::chrono::duration_cast<std::chrono::microseconds>(end -
//         start).count() / 1000.;
//         // printf("cost %2.4lf ms\n", tc);
//         cv::imshow("result", image);
//         cv::waitKey(1);
//     }
//     cv::destroyAllWindows();
//     delete yolov8;
//     return 0;
// }
