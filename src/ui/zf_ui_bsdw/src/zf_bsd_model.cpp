#include "zf_ui_bsdw/zf_bsd_model.h"

#include <math.h>

#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "rclcpp/serialization.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "zf_cam_model/zf_cam_model_truck.h"
#include "zf_framework_transport/transport.h"
#include "zf_global/common/zf_global_trailer_param.h"
#include "zf_global/util/logger.h"
#include "zf_global/zf_global_topic_name.h"

BEGIN_NS_ZF_UI
static const char gk_sTorqueLimitTSC1[] = "EngRqedTorque_TorqueLimit";
static const char gk_sAccDemandXBR[] = "ExtlAccelerationDemand";
// static const int gk_iCanIdTSC1 = 0xC000080;
// static const int gk_iCanIdXBR = 0xC040B80;

LOGGER::FileLogger LOG_FILE_PLANNING;

Eigen::Matrix3d RotZ(double angle, bool is_degree = true) {
  if (is_degree) {
    angle = angle * M_PI / 180.0;
  }

  Eigen::AngleAxisd rotation(angle, Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d rotM = rotation.toRotationMatrix();

  return rotM;
}

void PlotPolyLine(cv::Mat &img, Eigen::Matrix2Xd &m, const cv::Scalar &color)
{
    cv::Mat p_cv;
    cv::eigen2cv(m, p_cv);
    p_cv.convertTo(p_cv, CV_32SC1);
    cv::transpose(p_cv, p_cv);
    std::vector<cv::Point> pts((cv::Point*)p_cv.data, (cv::Point*)p_cv.data + p_cv.rows);
    // LOG_DEBUG() << m;
    // LOG_DEBUG() << p_cv;
    // for (auto & p: pts)
    // {
    //   LOG_INFO() << p.x << ", " << p.y;
    // }
    // LOG_ERROR() ;
    cv::polylines(img, pts, false, color, 2);
}


void Eigen2CvPts(Eigen::Matrix2Xd &m, std::vector<cv::Point> &points)
{
    cv::Mat p_cv;
    cv::eigen2cv(m, p_cv);
    p_cv.convertTo(p_cv, CV_32SC1);
    cv::transpose(p_cv, p_cv);
    std::vector<cv::Point> pts((cv::Point*)p_cv.data, (cv::Point*)p_cv.data + p_cv.rows);
    points.swap(pts);
}

// Function to check for intersection between two line segments
inline cv::Point CheckIntersection(const cv::Point &p1, const cv::Point &p2, const cv::Point &p3, const cv::Point &p4) {
    double xdiff = p1.x - p2.x;
    double ydiff = p1.y - p2.y;
    double xdiff2 = p3.x - p4.x;
    double ydiff2 = p3.y - p4.y;

    double div = xdiff * ydiff2 - ydiff * xdiff2;
    if (div == 0) {
        return {0, 0};  // Lines do not intersect
    }
    double d[2] = {p1.x * p2.y - p1.y * p2.x, p3.x * p4.y - p3.y * p4.x};
    double x = (d[0] * xdiff2 - xdiff * d[1]) / div;
    double y = (d[0] * ydiff2 - ydiff * d[1]) / div;

    if (std::min(p1.x, p2.x) <= x && x <= std::max(p1.x, p2.x) &&
        std::min(p3.x, p4.x) <= x && x <= std::max(p3.x, p4.x)) {
        return {x, y};
    }
    return {0, 0};  // Intersection const Vector2D &is not within line segments
}

BsdwModel::BsdwModel() : m_pTruckCam(new NS_ZF_DETECTION::CameraModelTrcuk) {
  m_classSet = {{"pedestrian", 1},  // person
                {"bicycle", 2},     // bicycle
                {"car", 0},         // car
                {"motorbike", 3},  {"2W", 3},           {"truck", 4},  // truck
                {"bus", 5},        {"truck_or_bus", 5}, {"unknown", 6},
                {"undecided", 6},  {"VRU", 0},          {"non_VRU", 7}};
  m_postfix.push_back("cam2");  // left
  m_postfix.push_back("cam1");  // right
  m_lrImgId = ELrImg::RIGHT;
  m_fAngleArticulation = 0.0f;

  // rclcpp::init(0, nullptr); // must init in the main function so that ros2
  // param can be transfered. Create a static executor - faster but doesn't
  // allow creating nodes during runtime Use MultiThreadedExecutor for runtime
  // node creation.
  memset(m_bSensorSource, 0, sizeof(m_bSensorSource));
  m_bSensorSource[6] = true;
  rclcpp::executors::StaticSingleThreadedExecutor::SharedPtr rosExecutor;
  rosExecutor =
      std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
  m_RosNode = rclcpp::Node::make_shared("bsdw_ui_node");
  rosExecutor->add_node(m_RosNode);
  m_bEnableDebug = m_RosNode->declare_parameter<bool>("enable_debug", false);
  LOG_DEBUG() << "m_bEnableDebug IS" << m_bEnableDebug;
  rclcpp::QoS video_qos(50);
  video_qos.keep_last(50);
  // video_qos.reliable();
  video_qos.best_effort();  // EngSpeedAtPoint2
  video_qos.durability_volatile();
  std::function<void(const interface::msg::SystemState::SharedPtr)> cbkSys =
      std::bind(&BsdwModel::CallbackNodeState, this, std::placeholders::_1,
                ESysFlag::FLAG_SYSTEM);
  m_subSysState = m_RosNode->create_subscription<interface::msg::SystemState>(
      gk_uiSystemState, video_qos, cbkSys);
  std::function<void(const interface::msg::SystemState::SharedPtr)> cbkSysData =
      std::bind(&BsdwModel::CallbackNodeState, this, std::placeholders::_1,
                ESysFlag::FLAG_DATA);
  m_subSysState = m_RosNode->create_subscription<interface::msg::SystemState>(
      gk_uiDataSystemState, video_qos, cbkSysData);

  m_subBevData = m_RosNode->create_subscription<interface::msg::HmiBevData>(
      gk_uiBevData, video_qos,
      std::bind(&BsdwModel::CallbackBevData, this, std::placeholders::_1));
  m_subRadarStatusInfo = m_RosNode->create_subscription<std_msgs::msg::String>(
      "/can1/hal/to_ui_radar_status", video_qos,
      std::bind(&BsdwModel::CallbackRadarInfo, this, std::placeholders::_1));
  m_subStwInfo = m_RosNode->create_subscription<std_msgs::msg::String>(
      "/can0/hal/to_ui", video_qos,
      std::bind(&BsdwModel::CallbackStwInfo, this, std::placeholders::_1));
  std::string topicName =
      "/can0/hal/to_can_msg_data";  // + gk_halToCanMsgData; //:
                                    // /can0/hal/to_can_msg_data
  m_subXBRTSC1 = m_RosNode->create_subscription<can_msgs::msg::CanMsgData>(
      topicName, video_qos,
      std::bind(&BsdwModel::CallbackPncData, this, std::placeholders::_1));
  std::vector<std::string> sensorList = {"cam",   "campp", "srr",
                                         "srrpp", "ipm",   "ipmpp"};
  for (uint32_t i = 0; i < sensorList.size(); ++i) {
    topicName = "/" + sensorList.at(i) +
                gk_halSensorObjs;  // /hal/from_packed_radar_data
    // if (m_vCanInterfaces.at(i).compare("can0")) {
    //   topicName = "/vehicle_info";
    // }
    int index = i;
    std::function<void(const interface::msg::HmiAgent::ConstSharedPtr)> cbk =
        std::bind(&BsdwModel::CallbackGetSensorObjs, this,
                  std::placeholders::_1, index);
    auto sub = m_RosNode->create_subscription<interface::msg::HmiAgent>(
        topicName, video_qos, cbk);
    m_vSubSensorDatas.push_back(sub);
    LOG_DEBUG() << "m_vSubSensorDatas topicName " << topicName.c_str();
  }
  // place the spin of ROS2 (which is a blocking infinite loop) in a seperate
  // thread and detach it Unlike ROS1, we use std::thread for threading instead
  // of the QThread Might return to QThread in the future if I'll see game
  // changing flaws.
  std::thread executor_thread(std::bind(
      &rclcpp::executors::StaticSingleThreadedExecutor::spin, rosExecutor));
  executor_thread.detach();
  m_timerPncStatus = new QTimer(this);
  QObject::connect(m_timerPncStatus, SIGNAL(timeout()), this,
                   SLOT(UpdatePncStatus()));
  m_timerPncStatus->start(200);
  LOG_FILE_PLANNING.SetFileName("planning.log");
  // file.open("/home/nvidia/Music/log/2024-07/2024-07-10
  // 16:47:10/can/planning.log", std::fstream::out | std::fstream::app |
  // std::fstream::ate);
  m_lastStamp = m_RosNode->get_clock()->now().nanoseconds();
  InitReceiver(0);
  InitReceiver(1);
  // std::thread thread1(&BsdwModel::InitReceiver, this);
  // thread1.detach();
  // cv::Mat img(1920, 1080, CV_8UC3, cv::Scalar(255, 255, 255));
  // PlotEgoLine(img);
}

MsgUi BsdwModel::GetMsgUi() { return m_msg2ui; }

MsgPncUi BsdwModel::GetMsgPncUi() { return m_msgPnc2ui; }

BsdwModel::Uint8Vec BsdwModel::GetNode() { return m_vNode; }

BsdwModel::Uint8Vec2d BsdwModel::GetNode2d() { return m_vNode2d; }

std::string BsdwModel::GetStwInfo() { return m_stwInfo; }

bool BsdwModel::GetSensorSource(int id) { return m_bSensorSource[id]; }

std::vector<FusionObjListItem> BsdwModel::GetSourceAgent(int id) {
  std::lock_guard<std::mutex> guard(m_lockSource[id]);
  return m_vSourceAgents[id];
}

void BsdwModel::SetSensorSource(int id, bool checked) {
  m_bSensorSource[id] = checked;
}

void BsdwModel::CallbackPncData(
    const can_msgs::msg::CanMsgData::SharedPtr msg) {
  if (m_iMissingCount != 0) {
    m_iMissingCount = 0;
  }
  for (auto &sig : msg->sig_datas) {
    if (sig.sig_name == gk_sTorqueLimitTSC1) {
      m_msgPnc2ui.torqueLimit = sig.sig_data;
      break;
    }
    if (sig.sig_name == gk_sAccDemandXBR) {
      m_msgPnc2ui.acc = sig.sig_data;
    }
  }
  emit MsgSubPncData(false);
}

void BsdwModel::UpdatePncStatus() {
  if ((++m_iMissingCount) > 5) {
    emit MsgSubPncData(true);
    m_iMissingCount -= 5;
  }
}

/*
  std::string line = "no_attention 15 no danger agent 1000";
  std::stringstream data(line);
  std::string guide;
  std::string id;
  int num;
  int size;

  data >> guide;
  data >> size;
  data.ignore(std::numeric_limits<std::streamsize>::max(), ' '); // Ignore the
  space id.resize(size); data.read(&id[0], size);
  //std::getline(data, id, ',');
  data >> num;
  */
void BsdwModel::CallbackBevData(
    const interface::msg::HmiBevData::SharedPtr msg) {
  m_msg2ui.time_stamp = msg->time_stamp;
  m_msg2ui.articulation_angle = Round(msg->articulation_angle * (-180) / M_PI);
  m_msg2ui.yaw_rate = Round(msg->yaw_rate);
  m_msg2ui.speed = Round(msg->speed);
  m_msg2ui.turn_light_lr = static_cast<uint16_t>(msg->turn_light_lr);
  m_msg2ui.ica_action_guide = msg->ica_action_guide;
  m_msg2ui.ica_trigger_ag_id = msg->ica_trigger_ag_id;
  m_msg2ui.ica_target_time = Round(msg->ica_target_time);
  m_msg2ui.ica_target_speed = Round(msg->ica_target_speed);
  m_msg2ui.agents.clear();
  m_msg2ui.ica_downgrade_reason = msg->ica_downgrade_reason;
  m_msg2ui.bsd_downgrade_reason = msg->bsd_downgrade_reason;
  m_msg2ui.driver_triggers_ica = msg->driver_triggers_ica;
  m_msg2ui.ica_control_flg = msg->ica_control_flg;

  std::replace(m_msg2ui.ica_trigger_ag_id.begin(),
               m_msg2ui.ica_trigger_ag_id.end(), ' ', '_');
  std::stringstream ss;
  auto time_list = msg->time_str;
  for (auto &stamp : time_list) {
    ss << stamp << " ";
  }
  ss << msg->time_stamp << " " << m_msg2ui.articulation_angle << " "
     << m_msg2ui.yaw_rate << " " << m_msg2ui.speed << " "
     << m_msg2ui.turn_light_lr << " " << msg->ica_action_guide << " "
     << m_msg2ui.ica_trigger_ag_id << " " << m_msg2ui.ica_target_time << " "
     << m_msg2ui.ica_target_speed << " ";
  auto agentCnt = msg->agents.size();
  ss << msg->agents.size() << " ";
  if (agentCnt) {
    ss << (msg->agents[0].agent.size() + 1) << " ";
  }
  for (auto &agt : msg->agents) {
    std::vector<std::string> obj = agt.agent;
    // for (const auto &agentStr : obj)
    // {
    //   ss << agentStr << " ";
    // }
    float deg = std::stod(obj[7]) * 180 / M_PI;
    FusionObjListItem agent = {
        QString::fromStdString(obj[0]),     // QString type, class
        std::stoi(obj[1]),                  // int bsdlevel
        std::stoi(obj[2]),                  // int icastatus;
        Round(std::stod(obj[3])),           // double posx;
        Round(std::stod(obj[4])),           // double posy;
        Round(std::stod(obj[5])),           // double width;
        Round(std::stod(obj[6])),           // double length;
        static_cast<int>(std::round(deg)),  // int heading;
        Round(std::stod(obj[8])),           // double velx;
        Round(std::stod(obj[9])),           // double vely;
        QString::fromStdString(obj[10]),    // QString name; fused_id
        m_classSet[obj[0]]                  // int itype;
    };
    ss << obj[0] << " "  //  agent.type.toStdString()
       << agent.bsdlevel << " " << agent.icastatus << " " << agent.posx << " "
       << agent.posy << " " << agent.width << " " << agent.length << " "
       << agent.heading << " " << agent.velx << " " << agent.vely << " "
       << obj[10] << " " << agent.itype << " ";
    m_msg2ui.agents.push_back(agent);
  }
  emit MsgSubBevData();
  if (m_bEnableDebug) {
    LOG_FILE_PLANNING() << ss.str();
    // rclcpp::Serialization<interface::msg::HmiBevData> planSerializer;
    // rclcpp::SerializedMessage serialized_msg;
    // planSerializer.serialize_message(msg.get(), &serialized_msg);
    // const auto buffer_begin =
    // serialized_msg.get_rcl_serialized_message().buffer; const auto buffer_end
    // = buffer_begin + serialized_msg.size(); std::string
    // serialized_msg_str(buffer_begin, buffer_end); LOG_WARN() <<
    // "serialized_msg_str is: " << ss.str(); std::string news =
    // serialized_msg_str;

    // std::ofstream file; // Replace "serialized_msg.txt" with the desired file
    // path if (file.is_open()) {
    //     file << serialized_msg_str;
    //     file.flush();
    //     std::cout << "Serialized message saved to file." << std::endl;
    // } else {
    //     std::cout << "Unable to open the file." << std::endl;
    // }
  }
  m_fAngleArticulation = m_msg2ui.articulation_angle;
  std::lock_guard<std::mutex> guard(m_lockSource[ESensorSource::FUSION]);
  m_vSourceAgents[ESensorSource::FUSION] = m_msg2ui.agents;
}

size_t BsdwModel::ReadStringsFromFile(const std::string &filePath) {
  std::ifstream file(filePath);
  if (file.is_open()) {
    std::string line;
    while (std::getline(file, line)) {
      m_offLineStrings.push_back(line);
    }
    file.close();
    LOG_DEBUG() << "Strings read from file: " << filePath;
  } else {
    LOG_ERROR() << "Failed to open the file: " << filePath;
  }
  return m_offLineStrings.size();
}

void BsdwModel::DeserializeLine(const std::string &line) {
  MsgUi msg;
  std::stringstream data(line);
  data >> msg.time_stamp >> msg.articulation_angle >> msg.yaw_rate >>
      msg.speed >> msg.turn_light_lr >> msg.ica_action_guide >>
      msg.ica_trigger_ag_id >> msg.ica_target_time >> msg.ica_target_speed;
  size_t agentCount, agentDataSize;
  data >> agentCount;
  data >> agentDataSize;
  for (size_t i = 0; i < agentCount; ++i) {
    std::string classType;
    std::string name;
    FusionObjListItem agent;
    data >> classType >> agent.bsdlevel >> agent.icastatus >> agent.posx >>
        agent.posy >> agent.width >> agent.length >> agent.heading >>
        agent.velx >> agent.vely >> name >> agent.itype;
    agent.type = QString::fromStdString(classType);
    agent.name = QString::fromStdString(name);
    // std::vector<std::string> obj;
    // for (size_t j = 0; j < agentDataSize; ++j)
    // {
    //   std::string agentStr;
    //   data >> agentStr;
    //   obj.push_back(agentStr);
    // }
    // float deg = std::stod(obj[7]) * 180 / M_PI;
    // FusionObjListItem agent = {
    //     QString::fromStdString(obj[0]),    // QString type, class
    //     std::stoi(obj[1]),                 // int bsdlevel
    //     std::stoi(obj[2]),                 // int icastatus;
    //     Round(std::stod(obj[3])),          // double posx;
    //     Round(std::stod(obj[4])),          // double posy;
    //     Round(std::stod(obj[5])),          // double width;
    //     Round(std::stod(obj[6])),          // double length;
    //     static_cast<int>(std::round(deg)), // int heading;
    //     Round(std::stod(obj[8])),          // double velx;
    //     Round(std::stod(obj[9])),          // double vely;
    //     QString::fromStdString(obj[10]),   // QString name; fused_id
    //     m_classSet[obj[0]]                 // int itype;
    // };
    msg.agents.push_back(agent);
  }
  m_msg2ui = msg;
  emit MsgSubBevData();
}

std::string BsdwModel::SerializeData(const MsgUi &msg) {
  std::stringstream ss;
  ss << msg.time_stamp << " " << msg.articulation_angle << " " << msg.yaw_rate
     << " " << msg.speed << " " << msg.turn_light_lr << " "
     << msg.ica_action_guide << " " << msg.ica_trigger_ag_id << " "
     << msg.ica_target_time << " " << msg.ica_target_speed << " ";
  auto agentCnt = msg.agents.size();
  ss << agentCnt << " ";
  // if (agentCnt)
  // {
  //   ss << msg.agents.agent[0].size() << " ";
  // }
  // for (auto &agt : agents)
  // {
  //   for (const auto &agentStr : agt)
  //   {
  //     ss << agentStr << " ";
  //   }
  // }

  return ss.str();
}

void BsdwModel::CallbackNodeState(
    const interface::msg::SystemState::SharedPtr msg, const ESysFlag &flag) {
  m_vNode.clear();
  // "CPU   %1\n"   "GPU   %2\n"  "MEM   %3\n" DISK  %4"
  Uint8Vec tNode = {msg->cpu_state,  msg->gpu_state, msg->memory_state,
                    msg->disk_state, msg->bsd_state, msg->ica_state};
  m_vNode.swap(tNode);
  m_vNode2d.clear();
  m_vNode2d.push_back(msg->watchdog_state_list);
  m_vNode2d.push_back(msg->node_state_list);
  m_dataSysState = msg->trigger_state_hmi;
  m_msgPnc2ui.time_stamp =
      std::to_string(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
  emit MsgSubSystemState(flag);
}

void BsdwModel::CallbackStwInfo(const std_msgs::msg::String::SharedPtr msg) {
  m_stwInfo = msg->data;
  emit MsgSubStwData();
}

void BsdwModel::CallbackRadarInfo(const std_msgs::msg::String::SharedPtr msg) {
  std::stringstream radarStatus(msg->data);
  radarStatus >> m_msgPnc2ui.stB0 >> m_msgPnc2ui.stC0;
}

void BsdwModel::CallbackGetSensorObjs(
    const interface::msg::HmiAgent::ConstSharedPtr msg, int index) {
  // m_vSourceAgents[index] =
  std::vector<FusionObjListItem> agts;
  for (auto &agt : msg->agent) {
    std::stringstream data(agt);
    std::string classType;
    std::string name;
    FusionObjListItem agent;
    data >> classType >> agent.bsdlevel >> agent.icastatus >> agent.posx >>
        agent.posy >> agent.width >> agent.length;
    double rad;
    data >> rad;
    float deg = rad * 180 / M_PI;
    data >> agent.velx >> agent.vely >> name;
    agent.type = QString::fromStdString(classType);
    agent.name = QString::fromStdString(name);
    agent.heading = static_cast<int>(deg);
    agent.posx = Round(agent.posx);
    agent.posy = Round(agent.posy);
    agent.width = Round(agent.width);
    agent.length = Round(agent.length);
    agent.velx = Round(agent.velx);
    agent.vely = Round(agent.vely);
    agent.itype = m_classSet[classType];
    agts.push_back(agent);
  }
  std::lock_guard<std::mutex> guard(m_lockSource[index]);
  m_vSourceAgents[index].swap(agts);
}

void BsdwModel::PlotAgents(cv::Mat &img) {
  std::vector<zf::ui::FusionObjListItem> agts;
  for (size_t i = 0; i <= ESensorSource::FUSION; ++i) {
    if (this->GetSensorSource(i)) {
      agts = this->GetSourceAgent(i);
      for (const auto &row : agts) {
        double posx = row.posx - 3.83;
        double posy = row.posy;
        double length = row.length;
        double width = row.width;
        double a = row.heading;
        std::string prop_class = row.type.toStdString();
        double vely = row.vely;
        double velx = row.velx;
        std::string sId = row.name.toStdString();

        Eigen::Matrix3d R = RotZ(a, false);
        Eigen::Vector3d cneter(posx, posy, 0);

        Eigen::Vector3d top_left(posx - length / 2, posy - width / 2, 0);
        Eigen::Vector3d top_right(posx + length / 2, posy - width / 2, 0);
        Eigen::Vector3d bottom_left(posx - length / 2, posy + width / 2, 0);
        Eigen::Vector3d bottom_right(posx + length / 2, posy + width / 2, 0);
        top_left = R * (top_left - cneter) + cneter;
        top_right = R * (top_right - cneter) + cneter;
        bottom_left = R * (bottom_left - cneter) + cneter;
        bottom_right = R * (bottom_right - cneter) + cneter;
        Eigen::Vector3d speed(posx + velx, posy + vely, 0);
        m_pTruckCam->SetRadarHeightInGround(0);
        Eigen::Vector2d pts0[4] = {m_pTruckCam->Radar2Pixel(top_left),
                                   m_pTruckCam->Radar2Pixel(top_right),
                                   m_pTruckCam->Radar2Pixel(bottom_right),
                                   m_pTruckCam->Radar2Pixel(bottom_left)};

        m_pTruckCam->SetRadarHeightInGround(1.6);  // height is max 1.6m
        Eigen::Vector2d ptsMax[4] = {m_pTruckCam->Radar2Pixel(top_left),
                                     m_pTruckCam->Radar2Pixel(top_right),
                                     m_pTruckCam->Radar2Pixel(bottom_right),
                                     m_pTruckCam->Radar2Pixel(bottom_left)};
        m_pTruckCam->SetRadarHeightInGround(0.505);  // height is max 1.6m

        cv::Mat emptyImg = cv::Mat::zeros(img.size(), img.type());
        std::vector<cv::Point> rectImg0Pts = {
            cv::Point(pts0[0].x(), pts0[0].y()),
            cv::Point(pts0[1].x(), pts0[1].y()),
            cv::Point(pts0[2].x(), pts0[2].y()),
            cv::Point(pts0[3].x(), pts0[3].y())};
        std::vector<cv::Point> rectImgMaxPts = {
            cv::Point(pts0[0].x(), pts0[0].y()),
            cv::Point(pts0[1].x(), pts0[1].y()),
            cv::Point(pts0[2].x(), pts0[2].y()),
            cv::Point(pts0[3].x(), pts0[3].y())};

        const cv::Point *p0[1] = {&rectImg0Pts[0]};
        const cv::Point *pMax[1] = {&rectImgMaxPts[0]};
        int numberOfPoints = (int)rectImg0Pts.size();
        int num_points = 4;
        cv::fillPoly(emptyImg, p0, &num_points, 1, CV_COLOR_BLUE);
        cv::fillPoly(emptyImg, pMax, &num_points, 1, CV_COLOR_BLUE);

        // Draw lines and polygons
        for (int i = 0; i < 4; i++) {
          cv::line(img, rectImg0Pts[i], rectImg0Pts[(i + 1) % 4], CV_COLOR_RED,
                   2);
          std::vector<cv::Point> face = {
              rectImg0Pts[i], rectImg0Pts[(i + 1) % 4],
              rectImgMaxPts[(i + 1) % 4], rectImgMaxPts[i]};
          const cv::Point *pFace[1] = {&face[0]};
          cv::fillPoly(emptyImg, pFace, &num_points, 1, CV_COLOR_BLUE);
          cv::line(img, rectImg0Pts[i], rectImgMaxPts[i], CV_COLOR_RED, 2);
        }
        cv::polylines(emptyImg, rectImg0Pts, true, CV_COLOR_RED, 2);
        cv::polylines(emptyImg, rectImgMaxPts, true, CV_COLOR_RED, 2);
        cv::addWeighted(img, 1, emptyImg, 0.5, 0, img);

        // Draw arrowed line
        cv::arrowedLine(img, cv::Point2f(cneter.x(), cneter.y()),
                        cv::Point2f(speed.x(), speed.y()), CV_COLOR_DEEPPINK,
                        2);
        // Draw ID text
        cv::putText(img, sId, rectImgMaxPts[0], cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    CV_COLOR_DEEPPINK, 2, cv::LINE_AA);
        cv::putText(img, prop_class, rectImg0Pts[2], cv::FONT_HERSHEY_SIMPLEX,
                    0.8, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
      }
    }
  }
}

void BsdwModel::PlotEgoLine(cv::Mat &img) {
  Eigen::MatrixXd trailer_corners =
      GlobalParams::Instance().GetCornersTrailer();
  int number = 10;
  Eigen::MatrixXd trailer_edges =
      GlobalParams::Instance().GetLinSpacePoints(trailer_corners, number);
  Eigen::Matrix3d R = RotZ(m_fAngleArticulation, true);
  Eigen::MatrixXd trailer_self = R * trailer_edges;  // 3Xn (40)
  Eigen::MatrixXd trailer_high =
      R * (trailer_edges +
           GlobalParams::Instance().GetLinSpacePoints(
               GlobalParams::Instance().GetTrailerOffestHigh(), number));
  Eigen::MatrixXd trailer_mid =
      R * (trailer_edges +
           GlobalParams::Instance().GetLinSpacePoints(
               GlobalParams::Instance().GetTrailerOffestMid(), number));
  Eigen::MatrixXd trailer_low =
      R * (trailer_edges +
           GlobalParams::Instance().GetLinSpacePoints(
               GlobalParams::Instance().GetTrailerOffestLow(), number));
  Eigen::MatrixXd head_corners = GlobalParams::Instance().GetCornersHead();
  Eigen::MatrixXd head_edges =
      GlobalParams::Instance().GetLinSpacePoints(head_corners, number);
  Eigen::MatrixXd head_high =
      head_edges + GlobalParams::Instance().GetLinSpacePoints(
                       GlobalParams::Instance().GetHeadOffestHigh(), number);
  Eigen::MatrixXd head_mid =
      head_edges + GlobalParams::Instance().GetLinSpacePoints(
                       GlobalParams::Instance().GetHeadOffestMid(), number);
  Eigen::MatrixXd head_low =
      head_edges + GlobalParams::Instance().GetLinSpacePoints(
                       GlobalParams::Instance().GetHeadOffestLow(), number);
  std::vector<Eigen::MatrixXd> list_rect = {
      trailer_self, trailer_high, trailer_mid, trailer_low,
      head_edges,   head_high,    head_mid,    head_low};
  std::vector<cv::Scalar> list_color = {CV_COLOR_RED, CV_COLOR_RED,
                                        CV_COLOR_YELLOW, CV_COLOR_GREEN};

  for (int i = 0; i < list_rect.size() / 2; ++i) {
    Eigen::MatrixXd t_trailer = list_rect[i];
    Eigen::Matrix2Xd points =
        this->m_pTruckCam->Ground2Pixel(t_trailer);  // transpose
    Eigen::Matrix2Xd p1_trailer = points.leftCols(number);
    Eigen::Matrix2Xd p2_trailer = points.middleCols(number, number);
    Eigen::Matrix2Xd p3_trailer = points.middleCols(2 * number, number);
    Eigen::Matrix2Xd p4_trailer = points.middleCols(3 * number, number);
    Eigen::MatrixXd t_head = list_rect[i + 4];
    Eigen::Matrix2Xd points_head = this->m_pTruckCam->Ground2Pixel(t_head);
    Eigen::Matrix2Xd p1_head = points_head.leftCols(number);
    Eigen::Matrix2Xd p2_head = points_head.middleCols(number, number);
    Eigen::Matrix2Xd p3_head = points_head.middleCols(2 * number, number);
    Eigen::Matrix2Xd p4_head = points_head.middleCols(3 * number, number);
    
    std::vector<cv::Point> p1_trailer_cv;
    std::vector<cv::Point> p1_head_cv;
    Eigen2CvPts(p1_trailer, p1_trailer_cv);
    Eigen2CvPts(p1_head, p1_head_cv);
    bool intersectionFound = false;
    for (size_t i = 0; i < p1_trailer_cv.size() - 1; ++i) {
        if (intersectionFound) {
            break;
        }
        for (size_t j = p1_head_cv.size() - 1; j >0 ; --j) {
            cv::Point intersection = CheckIntersection(p1_trailer_cv[i], p1_trailer_cv[i+1], p1_head_cv[j], p1_head_cv[j-1]);
            if (intersection.x != 0 || intersection.y != 0) {
                intersectionFound = true;
                // std::cout << "(" << i << ", " << j << ") ";
                p1_trailer_cv.erase(p1_trailer_cv.begin(), p1_trailer_cv.begin() + i);
                *p1_trailer_cv.begin() = intersection;
                p1_head_cv.erase(p1_head_cv.begin() + j, p1_head_cv.end());
                p1_head_cv.push_back(intersection);
                break;
            }
        }
    }
    cv::polylines(img, p1_trailer_cv, false, list_color.at(i), 2);
    cv::polylines(img, p1_head_cv, false, list_color.at(i), 2);

    // PlotPolyLine(img, p1_trailer, list_color.at(i));
    // PlotPolyLine(img, p1_head, list_color.at(i));
    PlotPolyLine(img, p2_trailer, list_color.at(i));
    PlotPolyLine(img, p4_head, list_color.at(i));

    // cv::Mat p1_trailer_cv;
    // cv::eigen2cv(p1_trailer, p1_trailer_cv);
    // p1_trailer_cv.convertTo(p1_trailer_cv, CV_32SC1);

    // cv::Mat p1_head_cv;
    // cv::eigen2cv(p1_head, p1_head_cv);
    // p1_head_cv.convertTo(p1_head_cv, CV_32SC1);
    // // LOG_DEBUG() << p1_head;
    // // LOG_DEBUG() << p1_head_cv;
    // std::vector<cv::Point> p1_trailer_((cv::Point*)p1_trailer_cv.data, (cv::Point*)p1_trailer_cv.data + p1_trailer_cv.cols);
    // std::vector<cv::Point> p1_head_((cv::Point*)p1_head_cv.data, (cv::Point*)p1_head_cv.data + p1_head_cv.cols);


    // cv::polylines(img, p1_trailer_, true, CV_COLOR_RED, 2);
    // cv::polylines(img, p1_head_, true, CV_COLOR_RED, 2);

    // Perform the necessary operations using Eigen matrices
  }
}
void BsdwModel::InitReceiver(const uint32_t &camId) {
  NS_ZF_FRAMEWORK::RoleAttributes attr;
  attr.SetHostIp(NS_ZF::GlobalData::Instance().HostIp());
  attr.SetChannelName(
      gk_channelDetToUi +
      m_postfix[camId]);  // gk_channelDetToUi gk_channelCamToDet
  NS_ZF_FRAMEWORK::Identity id;
  attr.SetId(id.HashValue());

  auto listener = [this, camId](
                      const std::shared_ptr<NS_ZF_FRAMEWORK::MsgBase> &msg,
                      const NS_ZF_FRAMEWORK::MessageInfo &msg_info,
                      const NS_ZF_FRAMEWORK::RoleAttributes &attr) {
    (void)msg_info;
    if (m_lrImgId != camId) {
      return;
    }
    // struct timespec ts;
    // clock_gettime(CLOCK_REALTIME, &ts);

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

    // std::vector<uint8_t> buffer(msg->Data().begin(), msg->Data().end());
    // char *raw_memory = (const_cast<char *>(msg->Data().data()));

    // std::vector<uint8_t> buffer(raw_memory, raw_memory + msg->Data().size());
    // cv::Mat decodedImage = cv::imdecode(cv::Mat(buffer), cv::IMREAD_COLOR);
    // cv::Mat decodedImage(1080, 1920, CV_8UC3, raw_memory);
    // cv::cvtColor(decodedImage, decodedImage, cv::COLOR_BGR2RGB);

    char *raw = (const_cast<char *>(msg->Data().data()));
    // std::vector<uint8_t> buffer(raw_memory, raw_memory + msg->Data().size());
    cv::Mat decodedImage(1080, 1920, CV_8UC3, raw);
    // cv::Mat decodedImage = rawImage.clone();
    // this->PlotAgents(decodedImage);

    cv::cvtColor(decodedImage, decodedImage, cv::COLOR_BGR2RGB);
    std::vector<cv::String> text;
    rclcpp::Time curr_stamp = m_RosNode->get_clock()->now();
    long long nanoseconds = curr_stamp.nanoseconds();

    text.push_back(cv::format("send: %lld.%lld",
                              msg->TimeStamp() / 1000000000LL,
                              msg->TimeStamp() % 1000000000LL));
    text.push_back(cv::format("recv: %lld.%lld", nanoseconds / 1000000000LL,
                              nanoseconds % 1000000000LL));
    text.push_back(
        cv::format("delay: %lld", (nanoseconds - msg->TimeStamp()) / 1000LL));
    text.push_back(cv::format("fps: %f", 1e9 / (nanoseconds - m_lastStamp)));
    // text.push_back(cv::format("from: %d", msg->TimeStamp()));
    m_lastStamp = nanoseconds;
    this->DrawTextLine(decodedImage, cv::Point(200, 130), text);
    // this->PlotEgoLine(decodedImage);
    // decodedImage = decodedImage.clone();
    m_img = QImage((uchar *)decodedImage.data, decodedImage.cols,
                   decodedImage.rows, decodedImage.step, QImage::Format_RGB888)
                .copy();  //.rgbSwapped()

    // const unsigned char *raw_memory =
    //     reinterpret_cast<const unsigned char *>(msg->Data().data());
    // m_img = QImage(raw_memory, 1920, 1080,
    // QImage::Format_RGB888); //.copy

    emit ImgUpdate();
    // std::chrono::steady_clock::time_point end =
    //     std::chrono::steady_clock::now();

    // std::cout
    //     // << "Time imdecode = " <<
    //     // std::chrono::duration_cast<std::chrono::microseconds>(end -
    //     // begin).count() << "[µs]"
    //     << "I heared: " << msg->Data().size()
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
    // // LOG_INFO() << "-------I heared: "  << total_name.c_str();
    // cv::imwrite(total_name, decodedImage, {cv::IMWRITE_JPEG_QUALITY, 50});

    // cv::namedWindow("OPENCV_WINDOW");
    // cv::Size newSize(640, 480); // Set the desired new size
    // cv::resize(decodedImage, decodedImage, newSize);
    // cv::imshow("Image from Char Array", decodedImage); // threads is not
    // adapt for opencv imshow

    // rclcpp::SerializedMessage serialized_msg(msg->Data().size());
    // // Set the serialized message buffer
    // memcpy(serialized_msg.get_rcl_serialized_message().buffer,
    //        msg->Data().c_str(), msg->Data().size());
    // serialized_msg.get_rcl_serialized_message().buffer_length =
    // msg->Data().size();
    // // Set the serialized message size
    // // Use the serialized message
    // const rclcpp::SerializedMessage *serialized_msg_ptr = &serialized_msg;
    // using MessageT = sensor_msgs::msg::Image;
    // sensor_msgs::msg::Image img_msg;
    // auto serializer = rclcpp::Serialization<MessageT>();
    // serializer.deserialize_message(&serialized_msg, &img_msg);
    // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg,
    // sensor_msgs::image_encodings::BGR8);
    // // Create Window and display it
    // if (!(cv_ptr->image.empty()))
    // {
    //   cv::imshow("Image from Char Array", cv_ptr->image);
    // }
  };

  m_recvShmImg[camId] =
      NS_ZF_FRAMEWORK::Transport::Instance()
          .CreateReceiver<NS_ZF_FRAMEWORK::MsgBase>(
              attr, listener, NS_ZF_FRAMEWORK::OptionalMode::SHM);
}

void BsdwModel::DrawTextLine(cv::Mat &img, cv::Point point,
                             const std::vector<cv::String> &textLines) {
  float fontScale = 0.6;
  int thickness = 2;
  // int fontFace = cv::FONT_HERSHEY_SIMPLEX;
  int fontFace = cv::LINE_AA;

  // size_t pos = 0;
  // while ((pos = textLine.find("\n")) != std::string::npos) {
  //     textLines.push_back(textLine.substr(0, pos));
  //     textLine.erase(0, pos + 1);
  // }
  // textLines.push_back(textLine);

  int baseline = 0;
  for (size_t i = 0; i < textLines.size(); i++) {
    std::string text = textLines[i];
    if (!text.empty()) {
      cv::Size textSize =
          cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
      cv::Point drawPoint(point.x,
                          point.y + (textSize.height + 10 + baseline) * i);
      cv::putText(img, text, drawPoint, cv::FONT_HERSHEY_DUPLEX, fontScale,
                  cv::Scalar(0, 0, 255), thickness, fontFace);
    }
  }

  // cv::putText(decodedImage,
  //             cv::format("send: %d.%d", tc, int(1000 / tc),
  //                        output_stracks.size()),
  //             cv::Point(200, 30), 0, 0.6, cv::Scalar(0, 0, 255), 2,
  //             cv::LINE_AA);
}

void BsdwModel::SaveImg(const QImage &img) {
  std::string total_name;
  std::ostringstream convert;
  // Extract time stamp
  convert << "/home/nvidia/Pictures/camera/";
  convert << m_lastStamp / 1000000000LL << "_" << m_lastStamp % 1000000000LL;
  convert << ".jpg";
  total_name = convert.str();
  // LOG_INFO() << "-------I heared: "  << total_name.c_str();
  img.save(QString::fromStdString(total_name), "jpg", 50);
}

END_NS_ZF_UI