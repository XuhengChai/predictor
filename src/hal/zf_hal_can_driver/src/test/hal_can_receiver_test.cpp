#include "zf_hal_can_driver/client/fake_can/fake_can_client.h"
#include "zf_hal_can_driver/client/can_client_factory.h"
#include "zf_hal_can_driver/hal_can_receiver.h"

#include "gtest/gtest.h"

BEGIN_NS_ZF_DRIVER_CANBUS

class SensorRadar
{
};

// class CanReceiverTest : public ::testing::Test
// {
// };  // class receiver

// TEST(CanTest, FakeCan)
// {
//     CANCardParameter param;
//     param.brand = CANCardParameter::CANCardBrand::FAKE_CAN;
//     CanClientFactory::Instance().RegisterCanClients();
//     auto can_client = CanClientFactory::Instance().CreateCANClient(param);
//     CanReceiver<SensorRadar> receiver;
//     // use std::move since unique_ptr not allow = and copy constructor. I recommend Init(CanClientFactory::Instance().CreateCANClient(param), true);
//     receiver.Init(std::move(can_client), true);
//     EXPECT_EQ(receiver.Start(), ErrorCode::OK);
//     EXPECT_TRUE(receiver.IsRunning());
//     USleep(2000);
//     receiver.Stop();
//     EXPECT_FALSE(receiver.IsRunning());

//     // cyber::Init("can_receiver_test");
//     // can::FakeCanClient can_client;
//     // MessageManager<::apollo::canbus::ChassisDetail> pm;
//     // CanReceiver<::apollo::canbus::ChassisDetail> receiver;

//     // receiver.Init(&can_client, &pm, false);
//     // EXPECT_EQ(receiver.Start(), common::ErrorCode::OK);
//     // EXPECT_TRUE(receiver.IsRunning());
//     // receiver.Stop();
//     // EXPECT_FALSE(receiver.IsRunning());
//     // cyber::Clear();
// }

// TEST(CanTest, SocketCan)
// {
//     CANCardParameter param;
//     param.brand = CANCardParameter::CANCardBrand::SOCKET_CAN;
//     CanClientFactory::Instance().RegisterCanClients();

//     auto client = CanClientFactory::Instance().CreateCANClient(param);
//     client->Open();
//     std::vector<CanFrame> buf;
//     int32_t frame_num = MAX_CAN_RECV_FRAME_LEN;
//     auto ret = client->Receive(&buf, 10);
//     LOG_INFO() << "CanTest Receive =---" << int(ret) << " " << buf.size();
//     while (1)
//     {
//         std::vector<CanFrame> buf;
//         auto ret = client->Receive(&buf, 10);
//         // sCAN.Write(sCAN.frame);
//         USleep(100);
//     }
// }

TEST(CanTest, SocketCanRecv)
{
    CANCardParameter param;
    param.brand = CANCardParameter::CANCardBrand::SOCKET_CAN;
    param.enableFD = true;
    CanClientFactory::Instance().RegisterCanClients();

    CanReceiver<SensorRadar> receiver;
    receiver.Init(CanClientFactory::Instance().CreateCANClient(param), true);

    EXPECT_EQ(receiver.Start(), ErrorCode::OK);
    EXPECT_TRUE(receiver.IsRunning());
    USleep(200000);
    receiver.Stop();
    EXPECT_FALSE(receiver.IsRunning());

    // cyber::Init("can_receiver_test");
    // can::FakeCanClient can_client;
    // MessageManager<::apollo::canbus::ChassisDetail> pm;
    // CanReceiver<::apollo::canbus::ChassisDetail> receiver;

    // receiver.Init(&can_client, &pm, false);
    // EXPECT_EQ(receiver.Start(), common::ErrorCode::OK);
    // EXPECT_TRUE(receiver.IsRunning());
    // receiver.Stop();
    // EXPECT_FALSE(receiver.IsRunning());
    // cyber::Clear();
}

END_NS_ZF_DRIVER_CANBUS