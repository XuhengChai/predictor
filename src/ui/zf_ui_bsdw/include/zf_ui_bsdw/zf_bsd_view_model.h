#ifndef ZF_BSD_VIEW_MODEL_H
#define ZF_BSD_VIEW_MODEL_H

// #include <ctime>
// #include <chrono>
#include <QGuiApplication>
#include <QObject>
#include <QTimer>
#include <iomanip>

#include "zf_bsd_gui_view.h"
#include "zf_bsd_model.h"
#include "zf_bsd_obj_view.h"
#include "zf_global/in/zf_ui_global.h"

BEGIN_NS_ZF_UI

class PerceptionViewModel : public QObject {
  Q_OBJECT
 public:
 private:
  BSDMainView *m_guiView;
  FusionObjList *m_objView;
  QObject *m_rootObj;
  std::unordered_map<uint8_t, int> m_bsdIndex;
  std::unordered_map<int, QString> m_mapStRadar;
  BsdwModel *m_backendModel;
  QString Tm2DataTime(std::string timestamp) {
    std::time_t time = static_cast<std::time_t>(std::stod(timestamp));
    std::tm *tm = std::localtime(&time);
    std::ostringstream oss;
    oss << std::put_time(tm, "%Y-%m-%d %H:%M:%S");
    std::string::size_type pos = timestamp.find(".");
    if (pos == std::string::npos) {
      timestamp.clear();
    } else {
      oss << timestamp.substr(pos, 3);
    }
    return QString::fromStdString(oss.str());
  }
 public slots:
  void UpdateSystemState(uint flag) {
    BsdwModel::Uint8Vec node = m_backendModel->GetNode();
    BsdwModel::Uint8Vec2d node2d = m_backendModel->GetNode2d();
    switch (flag) {
      case BsdwModel::ESysFlag::FLAG_SYSTEM: {
        this->SetNodeStatusStr(node, node2d);
        this->SetBsdStatus(node[4]);
        this->SetIcaStatus(node[5]);
        break;
      }
      case BsdwModel::ESysFlag::FLAG_DATA: {
        MsgPncUi data = m_backendModel->GetMsgPncUi();
        this->SetTimeStamp(Tm2DataTime(data.time_stamp));
        uint8_t status = m_backendModel->GetDataSysState();

        if (m_mapStRadar.find(data.stB0) == m_mapStRadar.end() ||
            m_mapStRadar.find(data.stC0) == m_mapStRadar.end()) {
          this->SetDataStatusStr(node2d, QString::number(data.stB0),
                                 QString::number(data.stC0));
          status = 60;
        } else {
          this->SetDataStatusStr(node2d, m_mapStRadar.at(data.stB0),
                                 m_mapStRadar.at(data.stB0));
        }
        if (m_bsdIndex.find(status) == m_bsdIndex.end()) {
          status = 60;
        }
        if (data.stB0 != 1 || data.stC0 != 1) {
          status = 60;
        }
        this->SetIcaStatus(status);
        break;
      }
      default:
        break;
    }
  }
  void UpdateStwInfo() {
    auto data = m_backendModel->GetStwInfo();
    bool err = data.find("1") != std::string::npos;
    this->SetAbStatus(err);
    // this->SetTimeStamp(QString::fromStdString(data));
  }
  void UpdatePncMsg(bool timeout) {
    if (timeout) {
      this->SetPncInfo("NAN");
      return;
    }
    MsgPncUi data = m_backendModel->GetMsgPncUi();
    QString info = QString(
                       "%1\n"
                       "%2")
                       .arg(data.acc)
                       .arg(data.torqueLimit);
    this->SetPncInfo(info);
  }
  void UpdateBevMsg() {
    MsgUi data = m_backendModel->GetMsgUi();
    QString icaInfo;
    if (data.ica_action_guide.find("no_") != std::string::npos) {
      icaInfo = QString(
                    "%1\n"
                    "\n"
                    "\n"
                    "%2, %3")
                    .arg(QString::fromStdString(data.ica_action_guide))
                    .arg(data.driver_triggers_ica)
                    .arg(data.ica_control_flg);
    } else {
      icaInfo = QString(
                    "%1\n"
                    "%2\n"
                    "%3\n"
                    "%4, %5")
                    .arg(QString::fromStdString(data.ica_action_guide))
                    .arg(data.ica_target_time)
                    .arg(data.ica_target_speed)
                    .arg(data.driver_triggers_ica)
                    .arg(data.ica_control_flg);
    }

    this->SetIcaInfo(icaInfo);
    this->SetTurnLR(data.turn_light_lr);
    this->SetTimeStamp(Tm2DataTime(data.time_stamp));
    this->SetEgoStatusAngle(data.articulation_angle);
    this->SetVehicleSpeed(data.speed);
    // this->SetAbStatus(true);
    // std::vector<FusionObjListItem> m_list;
    // m_list.push_back({"veh", 1, 0, 7.5, 1.5, 2.3, 4.6, 30, 0, 0, "A
    // Masterpiece", 0}); m_list.push_back({"ped", 2, -1, 12.0, -13.0, 0.8, 0.8,
    // 180, 0, 0, "John Doe", 1});
    std::vector<zf::ui::FusionObjListItem> agts;
    if (m_backendModel->GetSensorSource(ESensorSource::FUSION)) {
      agts = data.agents;
    }
    for (size_t i = 0; i < ESensorSource::FUSION; ++i) {
      if (m_backendModel->GetSensorSource(i)) {
        auto t_agt = m_backendModel->GetSourceAgent(i);
        agts.insert(agts.end(), t_agt.begin(), t_agt.end());
      }
    }
    this->SetFusionObjList(agts);
  }
  void UpdateSensorChecked(QVariant checks, QVariant index) {
    // QList<QVariant> checkList = checks.value<QList<QVariant>>();
    int id = index.toInt();
    bool isChecked = checks.toBool();
    // LOG_DEBUG() << id << " UpdateSensorChecked "  << checkList[id].toBool();
    m_backendModel->SetSensorSource(id, isChecked);
  }
  void UpdateLrImg(QVariant lr) {
    // left: 0, right : 1
    m_backendModel->SetLrImg(static_cast<ELrImg>(lr.toInt()));
    // LOG_DEBUG() << " SetLrImg IS" << lr.toInt();
  }
  void UpdateImg() { m_guiView->setImage(m_backendModel->GetImg()); }

 public:
  using Uint8Vec = std::vector<uint8_t>;
  using Uint8Vec2d = std::vector<Uint8Vec>;
  PerceptionViewModel() : m_rootObj(nullptr) {
    m_guiView = new BSDMainView();
    m_objView = new FusionObjList();
    m_backendModel = new BsdwModel();
    std::vector<uint8_t> bsdList = {10, 20, 25, 30, 35, 40, 45, 60};
    m_mapStRadar = {{-2, "None"},
                    {-1, "NAN"},
                    {0, "Initializing"},
                    {1, "Fully Operational"},
                    {2, "Performance Limited"},
                    {3, "Temporary Fault - Pending Recovery"},
                    {4, "Permanent Error - Reset to Recover"},
                    {5, "Falling Asleep"},
                    {6, "Shutting Down"},
                    {7, "Power Save / Testbench"}};
    for (size_t i = 0; i < bsdList.size(); ++i) {
      m_bsdIndex[bsdList[i]] = static_cast<uint8_t>(i);
    }
    QObject::connect(m_backendModel, SIGNAL(MsgSubBevData()), this,
                     SLOT(UpdateBevMsg()));
    QObject::connect(m_backendModel, SIGNAL(MsgSubPncData(bool)), this,
                     SLOT(UpdatePncMsg(bool)));
    QObject::connect(m_backendModel, SIGNAL(MsgSubSystemState(uint)), this,
                     SLOT(UpdateSystemState(uint)));
    QObject::connect(m_backendModel, SIGNAL(MsgSubStwData()), this,
                     SLOT(UpdateStwInfo()));
    QObject::connect(m_backendModel, SIGNAL(ImgUpdate()), this,
                     SLOT(UpdateImg()));
  }

  void SetSlidesNum(int num) { m_guiView->setSlidesNum(num); }

  void InitConnect(QObject *rootObj) {
    m_rootObj = rootObj;
    // QList<QObject *> playerObjs =
    //     m_rootObj->findChildren<QObject *>("sliderSpinBoxPlayer");
    // QObject *playerObj = playerObjs[0];
    // QList<QObject *> playerStopBtnObjs =
    //     m_rootObj->findChildren<QObject *>("playerStopBtn");
    // QObject *playerStopBtnObj = playerStopBtnObjs[0];
    QObject *sensorWgt = m_rootObj->findChild<QObject *>("sensorWidget");
    // LOG_INFO() << "sensorWgt IS " <<
    // sensorWgt->property("colNames").toStringList()[0].toStdString();
    if (sensorWgt) {
      QObject::connect(sensorWgt,
                       SIGNAL(sensorCheckedChanged(QVariant, QVariant)), this,
                       SLOT(UpdateSensorChecked(QVariant, QVariant)));
    }
    QObject::connect(m_rootObj, SIGNAL(lrImgChanged(QVariant)), this,
                     SLOT(UpdateLrImg(QVariant)));
    this->SetIcaStatus(10);
  }

  FusionObjList *GetViewObj() { return m_objView; }

  BSDMainView *GetView() { return m_guiView; }

  void SetEgoStatusAngle(float val) { m_guiView->setEgoAngle(val); }

  float GetEgoStatusAngle() { return m_guiView->getEgoAngle(); }

  void SetTurnLR(int val) { m_guiView->setTurnLR(val); }

  int GetTurnLR() { return m_guiView->getTurnLR(); }

  void SetTimeStamp(const QString &val) { m_guiView->setTimeStamp(val); }

  void SetIcaInfo(const QString &val) { m_guiView->setIcaInfo(val); }

  void SetPncInfo(const QString &val) { m_guiView->setPncInfo(val); }

  void SetMaxSlides(int val) { m_guiView->setMaxSlides(val); }

  void SetBsdStatus(int val) { m_guiView->setBsdStatus(m_bsdIndex[val]); }

  void SetIcaStatus(int val) { m_guiView->setIcaStatus(m_bsdIndex[val]); }

  void SetAbStatus(bool val) { m_guiView->setAbStatus(val); }

  void SetVehicleSpeed(float val) { m_guiView->setSpeed(val); }

  void SetFusionObjList(const std::vector<FusionObjListItem> &items) {
    m_objView->SetValueList(items);
  }

  void SetDataStatusStr(const Uint8Vec2d &node2d, const QString &b0,
                        const QString &c0) {
    QString string2 = QString(
                          "camera1    %1 | %2\n"
                          "camera2    %3 | %4\n"
                          "sync       %5 | %6\n"
                          "ego        %7 | %8\n"
                          "srr: %9 | %10\n")
                          .arg(node2d[0][0], -2)
                          .arg(node2d[1][0], -2)
                          .arg(node2d[0][1], -2)
                          .arg(node2d[1][1], -2)
                          .arg(node2d[0][2], -2)
                          .arg(node2d[1][2], -2)
                          .arg(node2d[0][3], -2)
                          .arg(node2d[1][3], -2)
                          .arg(b0)
                          .arg(c0);

    //        QStringList strList;
    //        strList << string1 << string2;
    QVariantList variantList;
    variantList.append(string2);
    m_guiView->setNodesStatus(variantList);
  }

  void SetNodeStatusStr(const Uint8Vec &node, const Uint8Vec2d &node2d) {
    QString string1 = QString(
                          "CPU   %1\n"
                          "GPU   %2\n"
                          "MEM   %3\n"
                          "DISK  %4")
                          .arg(node[0], -2)
                          .arg(node[1], -2)
                          .arg(node[2], -2)
                          .arg(node[3]);

    QString string2 = QString(
                          "Perception    %1 | %2\n"
                          "Fusion        %3 | %4\n"
                          "Planning-BSDW %5 | %6\n"
                          "Planning-ICA  %7 | %8\n"
                          "PnC           %9 | %10\n"
                          "Ego state     %11 | %12\n"
                          "CAN           %13 | %14")
                          .arg(node2d[0][0], -2)
                          .arg(node2d[1][0], -2)
                          .arg(node2d[0][1], -2)
                          .arg(node2d[1][1], -2)
                          .arg(node2d[0][2], -2)
                          .arg(node2d[1][2], -2)
                          .arg(node2d[0][3], -2)
                          .arg(node2d[1][3], -2)
                          .arg(node2d[0][4], -2)
                          .arg(node2d[1][4], -2)
                          .arg(node2d[0][5], -2)
                          .arg(node2d[1][5], -2)
                          .arg(node2d[0][6], -2)
                          .arg(node2d[1][6], -2);

    //        QStringList strList;
    //        strList << string1 << string2;
    QVariantList variantList;
    variantList.append(string1);
    variantList.append(string2);
    m_guiView->setNodesStatus(variantList);
  }

  void test_data() {
    this->SetIcaInfo("no_attention \n ");
    this->SetTurnLR(1);
    this->SetTimeStamp(
        "88,2022-04-22 "
        "15:01,file:///D:/DL/data/20240312/1501/data_img/"
        "1710226787.568941848.jpg");
    this->SetEgoStatusAngle(6.5);
    this->SetVehicleSpeed(7.2f);
    this->SetBsdStatus(45);
    this->SetIcaStatus(25);
    this->SetAbStatus(true);
    Uint8Vec node = {30, 30, 50, 60};
    Uint8Vec2d node2d = {{30, 50, 30, 50, 30, 50, 60},
                         {30, 50, 30, 50, 30, 50, 60}};
    this->SetNodeStatusStr(node, node2d);

    std::vector<FusionObjListItem> m_list;
    m_list.push_back(
        {"veh", 1, 0, 7.5, 1.5, 2.3, 4.6, 30, 0, 0, "A Masterpiece", 0});
    m_list.push_back(
        {"ped", 2, -1, 12.0, -13.0, 0.8, 0.8, 180, 0, 0, "John Doe", 1});
    this->SetFusionObjList(m_list);
  }
};
END_NS_ZF_UI
#endif  // ZF_BSD_VIEW_MODEL_H
