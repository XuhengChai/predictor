

#include "zf_framework_transport/transport.h"
#include "zf_framework_transport/shm/condition_notifier.h"

#include <memory>
#include <typeinfo>
#include <opencv2/opencv.hpp>

#include "gtest/gtest.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

    using TransmitterPtr = std::shared_ptr<Transmitter<MsgBase>>;
using ReceiverPtr = std::shared_ptr<Receiver<MsgBase>>;

TEST(TransportTest, create_transmitter)
{

  RoleAttributes attr;
  attr.SetChannelName("create_transmitter");
  Identity id;
  attr.SetId(id.HashValue());

  // TransmitterPtr intra =
  //     Transport::Instance().CreateTransmitter<MsgBase>(
  //         attr, OptionalMode::INTRA);
  // EXPECT_EQ(typeid(*intra), typeid(IntraTransmitter<MsgBase>));

  TransmitterPtr shm =
      Transport::Instance().CreateTransmitter<MsgBase>(
          attr, OptionalMode::SHM);
  EXPECT_EQ(typeid(*shm), typeid(ShmTransmitter<MsgBase>));
}

TEST(TransportTest, create_receiver)
{
  RoleAttributes attr;
  attr.SetChannelName("create_receiver");
  Identity id;
  attr.SetId(id.HashValue());

  auto listener = [](const std::shared_ptr<MsgBase> &,
                     const MessageInfo &, const RoleAttributes &) {};

  // ReceiverPtr intra = Transport::Instance().CreateReceiver<MsgBase>(
  //     attr, listener, OptionalMode::INTRA);
  // EXPECT_EQ(typeid(*intra), typeid(IntraReceiver<MsgBase>));

  ReceiverPtr shm = Transport::Instance().CreateReceiver<MsgBase>(
      attr, listener, OptionalMode::SHM);
  EXPECT_EQ(typeid(*shm), typeid(ShmReceiver<MsgBase>));
}

TEST(TransportTest, send_receiver)
{

  RoleAttributes attr;
  attr.SetHostIp(NS_ZF::GlobalData::Instance().HostIp());
  attr.SetChannelName("send_receiver");
  Identity id;
  attr.SetId(id.HashValue());
  TransmitterPtr shmTrans =
      Transport::Instance().CreateTransmitter<MsgBase>(
          attr, OptionalMode::SHM);

  RoleAttributes attrRecv;
  attrRecv.SetHostIp(NS_ZF::GlobalData::Instance().HostIp());
  attrRecv.SetChannelName("send_receiver");
  Identity idRecv;
  attrRecv.SetId(idRecv.HashValue());

  auto listener = [](const std::shared_ptr<MsgBase> &msg,
                     const MessageInfo &msg_info, const RoleAttributes &attr)
  {
    (void)msg_info;
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    LOG_INFO() << "-------I heared: " << msg->Data()
               << " send tmp: " << msg->TimeStamp() / 1000000000LL
               << "." << msg->TimeStamp() % 1000000000LL
               << " recv tmp: " << ts.tv_sec << "." << ts.tv_nsec;
  };

  // ReceiverPtr intra = Transport::Instance().CreateReceiver<MsgBase>(
  //     attr, listener, OptionalMode::INTRA);
  // EXPECT_EQ(typeid(*intra), typeid(IntraReceiver<MsgBase>));

  ReceiverPtr shmRecv = Transport::Instance().CreateReceiver<MsgBase>(
      attrRecv, listener, OptionalMode::SHM);

  struct timespec ts;
  for (int i = 0; i < 100; ++i)
  {
    clock_gettime(CLOCK_REALTIME, &ts);
    long long nanoseconds = ts.tv_sec * 1000000000LL + ts.tv_nsec;
    auto send_msg = std::make_shared<MsgBase>("raw_message--" + std::to_string(i), nanoseconds);
    shmTrans->Transmit(send_msg);
    // LOG_INFO() << i << ": tmp: " << ts.tv_sec << "." << ts.tv_nsec;
  }
  sleep(1);
}

TEST(TransportTest, send_receiver_img)
{
  RoleAttributes attr;
  attr.SetHostIp(NS_ZF::GlobalData::Instance().HostIp());
  attr.SetChannelName("send_receiver_img");
  Identity id;
  attr.SetId(id.HashValue());
  TransmitterPtr shmTrans =
      Transport::Instance().CreateTransmitter<MsgBase>(
          attr, OptionalMode::SHM);

  RoleAttributes attrRecv;
  attrRecv.SetHostIp(NS_ZF::GlobalData::Instance().HostIp());
  attrRecv.SetChannelName("send_receiver_img");
  Identity idRecv;
  attrRecv.SetId(idRecv.HashValue());

  auto listener = [](const std::shared_ptr<MsgBase> &msg,
                     const MessageInfo &msg_info, const RoleAttributes &attr)
  {
    (void)msg_info;
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    LOG_INFO() << "-------I heared: " << msg->Data().size()
               << " send tmp: " << msg->TimeStamp() / 1000000000LL
               << "." << msg->TimeStamp() % 1000000000LL
               << " recv tmp: " << ts.tv_sec << "." << ts.tv_nsec;
  };

  // ReceiverPtr intra = Transport::Instance().CreateReceiver<MsgBase>(
  //     attr, listener, OptionalMode::INTRA);
  // EXPECT_EQ(typeid(*intra), typeid(IntraReceiver<MsgBase>));

  ReceiverPtr shmRecv = Transport::Instance().CreateReceiver<MsgBase>(
      attrRecv, listener, OptionalMode::SHM);
  cv::Mat image = cv::imread("/home/nvidia/Pictures/tran_data/1705648736_598814045.jpg", cv::IMREAD_COLOR);
  std::vector<uchar> buffer;
  cv::imencode(".jpg", image, buffer);
  // Convert the buffer to a std::string
  std::string encodedImage(buffer.begin(), buffer.end());

  for (int i = 0; i < 200; ++i)
  {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    // Convert the seconds to nanoseconds and add the nanoseconds component
    long long nanoseconds = ts.tv_sec * 1000000000LL + ts.tv_nsec;
    auto send_msg = std::make_shared<MsgBase>(encodedImage, nanoseconds);
    shmTrans->Transmit(send_msg);
    LOG_INFO() << i << ": tmp: " << ts.tv_sec << "." << ts.tv_nsec;
    // sleep(1);
    // std::this_thread::sleep_for(std::chrono::microseconds(30000));
  }
  sleep(1);
  LOG_INFO() << "DONE";
}

// TEST(TransportTest, shut_down)
// {
//   RoleAttributes attr;
//   attr.SetHostIp(NS_ZF::GlobalData::Instance().HostIp());
//   attr.SetChannelName("send_receiver_img");
//   Identity id;
//   attr.SetId(id.HashValue());
//   auto segment = SegmentFactory::CreateSegment(attr.GetChannelId());
//   WritableBlock wb;
//   // std::size_t msg_size = MsgBase::ByteSize(msg);
//   if (!segment->AcquireBlockToWrite(16, &wb))
//   {
//     LOG_ERROR() << "acquire block failed.";
//     return;
//   }
//   LOG_DEBUG() << "block index: " << wb.index;
//   segment->ReleaseWrittenBlock(wb);
//   segment->Destroy();
//   // auto &notifier = ConditionNotifier::Instance();
//   // notifier.Shutdown();
//   sleep(1);
// }

END_NS_ZF_FRAMEWORK

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  zf::framework::Transport::Instance();
  auto res = RUN_ALL_TESTS();
  zf::framework::Transport::Instance().Shutdown();
  return res;
}
