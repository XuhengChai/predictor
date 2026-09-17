#ifndef ZF_BSD_MODEL_H
#define ZF_BSD_MODEL_H
// #include <memory>
// #include <mutex>
#include <QGuiApplication>
#include <QImage>
#include <QObject>
#include <QTimer>
#include <opencv2/opencv.hpp>


#include "can_msgs/msg/can_msg_data.hpp"
#include "can_msgs/msg/can_sig_data.hpp"
#include "interface/msg/hmi_agent.hpp"
#include "interface/msg/hmi_bev_data.hpp"
#include "interface/msg/system_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "zf_global/in/zf_detect_global.h"
#include "zf_global/in/zf_framework_global.h"
#include "zf_global/in/zf_ui_global.h"
#include "zf_ui_struct.h"

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
class CameraModelTrcuk;
END_NS_ZF_DETECTION

BEGIN_NS_ZF_UI

struct MsgUi {
  std::string time_stamp;
  float articulation_angle;
  float yaw_rate;
  float speed;
  // TURN_LEFT = 0, TURN_RIGHT = 1,  STRAIGHT = 2
  uint16_t turn_light_lr;
  std::string ica_action_guide;
  std::string ica_trigger_ag_id;
  float ica_target_time;
  float ica_target_speed;
  std::vector<FusionObjListItem> agents;
  std::string bsd_downgrade_reason;
  std::string ica_downgrade_reason;
  int32_t driver_triggers_ica;
  int32_t ica_control_flg;
};

enum ESensorSource : uint32_t {
  CAM = 0,
  CAMPP = CAM + 1,
  SRR = CAMPP + 1,
  SRRPP = SRR + 1,
  IPM = SRRPP + 1,
  IPMPP = IPM + 1,
  FUSION = IPMPP + 1,
};

struct MsgPncUi {
  std::string time_stamp;
  float acc;
  float torqueLimit;
  int stB0 = -2;
  int stC0 = -2;
};
enum ELrImg : uint32_t { LEFT = 0, RIGHT = 1 };

class BsdwModel : public QObject {
  Q_OBJECT
 public:
  using Uint8Vec = std::vector<uint8_t>;
  using Uint8Vec2d = std::vector<Uint8Vec>;
  enum ESysFlag : uint { FLAG_DATA = 0, FLAG_SYSTEM = 1 };
  BsdwModel();
  ~BsdwModel() {};
  MsgUi GetMsgUi();
  MsgPncUi GetMsgPncUi();
  Uint8Vec GetNode();
  Uint8Vec2d GetNode2d();
  std::string GetStwInfo();
  bool GetSensorSource(int id);
  std::vector<FusionObjListItem> GetSourceAgent(int id);
  void SetSensorSource(int id, bool checked);
  void SetLrImg(ELrImg id) {m_lrImgId = id;};
  inline double Round(double num, int digits) {
    return std::floor(num * pow(10, digits) + 0.5) / pow(10, digits);
  }
  inline double Round(double num) { return std::floor(num * 100 + 0.5) / 100; }
  const QImage &GetImg() { return m_img; };//std::lock_guard<std::mutex> lock(mutex_);
  const uint8_t &GetDataSysState() { return m_dataSysState; };
 Q_SIGNALS:
  // used to tell the backend that new ros data arrived and to update the gui
  void MsgSubSystemState(uint flag);
  void MsgSubBevData();
  void MsgSubPncData(bool timeout);
  void MsgSubStwData();
  void ImgUpdate();
 public slots:
  void UpdatePncStatus();

 private:
  void CallbackBevData(const interface::msg::HmiBevData::SharedPtr msg);
  void CallbackPncData(const can_msgs::msg::CanMsgData::SharedPtr msg);
  // void LoadBevData();
  size_t ReadStringsFromFile(const std::string &filePath);
  void DeserializeLine(const std::string &line);
  std::string SerializeData(const MsgUi &msg);
  void CallbackNodeState(const interface::msg::SystemState::SharedPtr msg,
                         const ESysFlag &flag);
  void CallbackStwInfo(const std_msgs::msg::String::SharedPtr msg);
  void CallbackRadarInfo(const std_msgs::msg::String::SharedPtr msg);
  void CallbackGetSensorObjs(const interface::msg::HmiAgent::ConstSharedPtr msg,
                             int index);
  void PlotAgents(cv::Mat &img);
  void PlotEgoLine(cv::Mat &img);
  void InitReceiver(const uint32_t &camId);
  void DrawTextLine(cv::Mat &img, cv::Point point, const std::vector<cv::String> &textLine);
  void SaveImg(const QImage &img);

 private:
  QImage m_img;
  uint32_t m_lrImgId;
  long long m_lastStamp;
  std::string m_stwInfo;
  std::vector<std::string> m_postfix;
  Uint8Vec m_vNode;      //{30, 30, 50, 60};
  Uint8Vec2d m_vNode2d;  //{{30, 50, 30, 50, 30, 50, 60}, {30, 50, 30, 50, 30,
                         // 50, 60}};
  uint8_t m_dataSysState;
  double m_dataSysTime;
  MsgUi m_msg2ui;
  float m_fAngleArticulation;
  MsgPncUi m_msgPnc2ui;
  std::map<std::string, int> m_classSet;
  rclcpp::Node::SharedPtr m_RosNode;
  bool m_bEnableDebug;
  rclcpp::Subscription<interface::msg::HmiBevData>::SharedPtr m_subBevData;
  rclcpp::Subscription<interface::msg::SystemState>::SharedPtr m_subSysState;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subStwInfo;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subRadarStatusInfo;
  rclcpp::Subscription<can_msgs::msg::CanMsgData>::SharedPtr m_subXBRTSC1;
  std::vector<rclcpp::Subscription<interface::msg::HmiAgent>::SharedPtr>
      m_vSubSensorDatas;
  uint32_t m_iMissingCount = {};
  QTimer *m_timerPncStatus;
  // std::ofstream file;
  std::vector<std::string> m_offLineStrings;
  bool m_bSensorSource[7];
  std::vector<FusionObjListItem> m_vSourceAgents[7];
  std::mutex m_lockSource[7];
  NS_ZF_FRAMEWORK::ReceiverPtr m_recvShmImg[2];
  std::shared_ptr<NS_ZF_DETECTION::CameraModelTrcuk> m_pTruckCam;
};
END_NS_ZF_UI
#endif  // ZF_BSD_MODEL_H
