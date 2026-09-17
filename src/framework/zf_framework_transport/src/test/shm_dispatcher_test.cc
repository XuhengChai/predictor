/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "zf_framework_transport/dispatcher/shm_dispatcher.h"
#include "zf_framework_transport/transmitter/shm_transmitter.h"
#include "zf_global/common/zf_global_data.h"

#include <memory>
#include "gtest/gtest.h"
#include <opencv2/opencv.hpp>

BEGIN_NS_ZF_FRAMEWORK // zf::framework

TEST(ShmDispatcherTest, add_listener)
{
  RoleAttributes self_attr;
  self_attr.SetChannelName("add_listener");
  Identity self_id;
  self_attr.SetId(self_id.HashValue());

  ShmDispatcher::Instance().AddListener<MsgBase>(
      self_attr,
      [](const std::shared_ptr<MsgBase> &, const MessageInfo &) {});

  RoleAttributes oppo_attr;
  oppo_attr.SetChannelName("add_listener");
  Identity oppo_id;
  oppo_attr.SetId(oppo_id.HashValue());

  ShmDispatcher::Instance().AddListener<MsgBase>(
      self_attr,
      [](const std::shared_ptr<MsgBase> &, const MessageInfo &) {}, oppo_attr);
}

TEST(ShmDispatcherTest, on_message)
{
  RoleAttributes oppo_attr;
  oppo_attr.SetHostIp(NS_ZF::GlobalData::Instance().HostIp());
  oppo_attr.SetChannelName("on_message");
  Identity oppo_id;
  oppo_attr.SetId(oppo_id.HashValue());

  std::shared_ptr<Transmitter<MsgBase>> transmitter = std::make_shared<ShmTransmitter<MsgBase>>(oppo_attr);
  transmitter->Enable();
  EXPECT_NE(transmitter, nullptr);

  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  // Convert the seconds to nanoseconds and add the nanoseconds component
  long long nanoseconds = ts.tv_sec * 1000000000LL + ts.tv_nsec;

  auto send_msg = std::make_shared<MsgBase>("raw_message", nanoseconds);
  transmitter->Transmit(send_msg);
  LOG_INFO() << " send tmp: " << ts.tv_sec << "." << ts.tv_nsec;

  // sleep(1);

  RoleAttributes self_attr;
  self_attr.SetChannelName("on_message");
  Identity self_id;
  self_attr.SetId(self_id.HashValue());

  auto recv_msg = std::make_shared<MsgBase>();
  ShmDispatcher::Instance().AddListener<MsgBase>(
      self_attr, [&recv_msg](const std::shared_ptr<MsgBase> &msg,
                             const MessageInfo &msg_info)
      {
        (void)msg_info;
        recv_msg->SetData(msg->Data());
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        long long nanoseconds = ts.tv_sec * 1000000000LL + ts.tv_nsec;
        LOG_INFO() << "-------I heared: " << msg->Data()
        << " send tmp: "<< msg->TimeStamp()
        << " recv tmp: " << ts.tv_sec << "." << ts.tv_nsec; });
  clock_gettime(CLOCK_REALTIME, &ts);
  send_msg->SetData("second msg");
  transmitter->Transmit(send_msg);
  LOG_INFO() << " send tmp: " << ts.tv_sec << "." << ts.tv_nsec;

  sleep(1);
  EXPECT_EQ(recv_msg->Data(), send_msg->Data());
}

TEST(ShmDispatcherTest, on_img_message)
{
  RoleAttributes oppo_attr;
  oppo_attr.SetHostIp(NS_ZF::GlobalData::Instance().HostIp());
  oppo_attr.SetChannelName("on_img_message");
  Identity oppo_id;
  oppo_attr.SetId(oppo_id.HashValue());

  std::shared_ptr<Transmitter<MsgBase>> transmitter = std::make_shared<ShmTransmitter<MsgBase>>(oppo_attr);
  transmitter->Enable();
  EXPECT_NE(transmitter, nullptr);
  cv::Mat image = cv::imread("/home/nvidia/Pictures/tran_data/1705648736_598814045.jpg", cv::IMREAD_COLOR); 
  std::vector<uchar> buffer;
  cv::imencode(".jpg", image, buffer);
  // Convert the buffer to a std::string
  std::string encodedImage(buffer.begin(), buffer.end());
  
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  // Convert the seconds to nanoseconds and add the nanoseconds component
  long long nanoseconds = ts.tv_sec * 1000000000LL + ts.tv_nsec;
  auto send_msg = std::make_shared<MsgBase>(encodedImage, nanoseconds);
  transmitter->Transmit(send_msg);
  LOG_INFO() << encodedImage.size() << " send tmp: " << ts.tv_sec << "." << ts.tv_nsec;

  // sleep(1);

  RoleAttributes self_attr;
  self_attr.SetChannelName("on_img_message");
  Identity self_id;
  self_attr.SetId(self_id.HashValue());

  auto recv_msg = std::make_shared<MsgBase>();
  ShmDispatcher::Instance().AddListener<MsgBase>(
      self_attr, [&recv_msg](const std::shared_ptr<MsgBase> &msg,
                             const MessageInfo &msg_info)
      {
        (void)msg_info;
        recv_msg->SetData(msg->Data());
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        LOG_INFO() << "-------I heared: " << msg->Data().size()
        << " send tmp: "<< msg->TimeStamp()
        << " recv tmp: " << ts.tv_sec << "." << ts.tv_nsec; });
  clock_gettime(CLOCK_REALTIME, &ts);
  // send_msg->SetData("second msg");
  transmitter->Transmit(send_msg);
  LOG_INFO() << " send tmp: " << ts.tv_sec << "." << ts.tv_nsec;

  sleep(1);
  EXPECT_EQ(recv_msg->Data(), send_msg->Data());
}

TEST(ShmDispatcherTest, shutdown)
{
  ShmDispatcher::Instance().Shutdown();
  // repeated call
  ShmDispatcher::Instance().Shutdown();
}

END_NS_ZF_FRAMEWORK

// int main(int argc, char** argv) {
//   testing::InitGoogleTest(&argc, argv);
//   apollo::cyber::Init(argv[0]);
//   apollo::cyber::transport::Transport::Instance();
//   auto res = RUN_ALL_TESTS();
//   apollo::cyber::transport::Transport::Instance().Shutdown();
//   return res;
// }
