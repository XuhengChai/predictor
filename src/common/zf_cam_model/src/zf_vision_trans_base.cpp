#include "zf_cam_model/zf_vision_trans_base.h"

#include <cstdio>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "zf_framework_transport/transport.h"

BEGIN_NS_ZF_DETECTION

VisionTransBase::VisionTransBase()
    : m_pTruckCam(new CameraModelTrcuk) {
  m_bTrans2HMI = false;
  m_strSavePath = "/home/nvidia/Pictures/camera/";

  // InitTransmit();
  // InitReceiver();
  // std::thread thread1(&VisionTransBase::InitReceiver, this, 0);
  // thread1.detach();
}

VisionTransBase::~VisionTransBase() {}

void VisionTransBase::InitTransmit(const std::string &channel_name) {
  NS_ZF_FRAMEWORK::RoleAttributes attr;
  attr.SetHostIp(NS_ZF::GlobalData::Instance().HostIp());
  attr.SetChannelName(channel_name);
  NS_ZF_FRAMEWORK::Identity id;
  attr.SetId(id.HashValue());
  m_transmitterImg = NS_ZF_FRAMEWORK::Transport::Instance()
                         .CreateTransmitter<NS_ZF_FRAMEWORK::MsgBase>(
                             attr, NS_ZF_FRAMEWORK::OptionalMode::SHM);
}

void VisionTransBase::InitReceiver(const std::string &channel_name) {
  NS_ZF_FRAMEWORK::RoleAttributes attr;
  attr.SetHostIp(NS_ZF::GlobalData::Instance().HostIp());
  attr.SetChannelName(channel_name);
  NS_ZF_FRAMEWORK::Identity id;
  attr.SetId(id.HashValue());

  auto listener = [this](const std::shared_ptr<NS_ZF_FRAMEWORK::MsgBase> &msg,
                         const NS_ZF_FRAMEWORK::MessageInfo &msg_info,
                         const NS_ZF_FRAMEWORK::RoleAttributes &attr) {
    (void)msg_info;
    // TODO use thread to handle
    auto start = std::chrono::system_clock::now();

    char *raw_memory = (const_cast<char *>(msg->Data().data()));
    cv::Mat decodedImage(1080, 1920, CV_8UC3, raw_memory);
    cv::cvtColor(decodedImage, decodedImage, cv::COLOR_BGR2RGB);
    m_img = decodedImage.clone();
    this->DetectCbk(m_img, msg->TimeStamp());
    this->NodeStatusCbk();
    // cv::cvtColor(decodedImage, decodedImage, cv::COLOR_RGB2GRAY);
    if (m_bTrans2HMI) {
      char *decodedImage_memory = (reinterpret_cast<char *>(m_img.data));
      std::string encodedImage(decodedImage_memory, msg->Data().size());
      auto send_msg = std::make_shared<NS_ZF_FRAMEWORK::MsgBase>(
          encodedImage, msg->TimeStamp());  // int(m_postfix[camId].back())
      // msg->ParseFromString(encodedImage);
      m_transmitterImg->Transmit(send_msg);
    }

    if (m_bSaveImg) {
      std::string total_name;
      std::ostringstream convert;
      // Extract time stamp
      convert << m_strSavePath;
      convert << msg->TimeStamp() / 1000000000LL << "_"
              << msg->TimeStamp() % 1000000000LL;
      convert << ".jpg";
      total_name = convert.str();
      // LOG_INFO() << "-------I heared imwrite: "  << total_name.c_str();
      cv::imwrite(total_name, m_img, {cv::IMWRITE_JPEG_QUALITY, 50});
    }
  };

  m_recvShmImg = NS_ZF_FRAMEWORK::Transport::Instance()
                     .CreateReceiver<NS_ZF_FRAMEWORK::MsgBase>(
                         attr, listener, NS_ZF_FRAMEWORK::OptionalMode::SHM);
}

END_NS_ZF_DETECTION