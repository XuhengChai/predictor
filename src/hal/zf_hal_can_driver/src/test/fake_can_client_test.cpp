// #include "client/fake_can/fake_can_client.h"
#include "zf_hal_can_driver/client/fake_can/fake_can_client.h"
#include "gtest/gtest.h"

BEGIN_NS_ZF_DRIVER_CANBUS

class FakeCanClientTest : public ::testing::Test {
 public:
  static const int32_t FRAME_LEN = 10;

  virtual void SetUp() {
    send_time_ = 0;
    recv_time_ = 0;
    send_succ_count_ = 0;
    recv_succ_count_ = 0;
    send_err_count_ = 0;
    recv_err_count_ = 0;
    param_.brand = CANCardParameter::FAKE_CAN;
    param_.channelID = CANCardParameter::CHANNEL_ID_ZERO;
    send_client_ = std::make_unique<FakeCanClient>();
    send_client_->Init(param_);
    send_client_->Open();
    recv_client_ = std::make_unique<FakeCanClient>();
    recv_client_->Init(param_);
    recv_client_->Open();
  }

 protected:
  std::unique_ptr<FakeCanClient> send_client_;
  std::unique_ptr<FakeCanClient> recv_client_;

  int64_t send_time_ = 0;
  int64_t recv_time_ = 0;
  int32_t send_succ_count_ = 0;
  int32_t recv_succ_count_ = 0;
  int32_t send_err_count_ = 0;
  int32_t recv_err_count_ = 0;
  std::stringstream recv_ss_;
  CANCardParameter param_;
};

// TEST_F(FakeCanClientTest, SendMessage) {
//   std::vector<CanFrame> frames;
//   frames.resize(FRAME_LEN);
//   for (int32_t i = 0; i < FRAME_LEN; ++i) {
//     frames[i].id = 1 & 0x3FF;
//     frames[i].len = 8;
//     frames[i].data[7] = 1 % 256;
//     for (uint8_t j = 0; j < 7; ++j) {
//       frames[i].data[j] = j;
//     }
//   }

//   int32_t frame_num = FRAME_LEN;
//   auto ret = send_client_->Send(frames, frame_num);
//   EXPECT_EQ(ret, ErrorCode::OK);
//   EXPECT_EQ(send_client_->GetErrorString(0), "");
//   send_client_->Close();
// }

TEST_F(FakeCanClientTest, ReceiveMessage) {
  std::vector<CanFrame> buf;
  int32_t frame_num = FRAME_LEN;

  auto ret = recv_client_->Receive(&buf, frame_num);
  EXPECT_EQ(ret, ErrorCode::OK);
  recv_client_->Close();
}

END_NS_ZF_DRIVER_CANBUS