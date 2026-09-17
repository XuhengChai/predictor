#include "zf_hal_can_driver/client/fake_can/fake_can_client.h"
#include <thread>

/**
 * @namespace zf::driver::canbus
 */
BEGIN_NS_ZF_DRIVER_CANBUS

bool FakeCanClient::Init(const CANCardParameter &) { return true; }

ErrorCode FakeCanClient::Open() { return ErrorCode::OK; }

void FakeCanClient::Close() {}

ErrorCode FakeCanClient::Send(const std::vector<CanFrame> &frames,
                              const int32_t &frame_num,
                              const std::chrono::nanoseconds &)
{

  if (static_cast<size_t>(frame_num) != frames.size())
  {
    LOG_ERROR() << "frame num is incorrect.";
    return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
  }
  for (size_t i = 0; i < frames.size(); ++i)
  {
    LOG_DEBUG() << "send frame i:" << i;
    LOG_DEBUG() << frames[i].CanFrameString();
    frame_info_ << frames[i].CanFrameString();
  }
  ++send_counter_;
  return ErrorCode::OK;
}

ErrorCode FakeCanClient::Receive(std::vector<CanFrame> *const frames,
                                 const int32_t &frame_num,
                                 const std::chrono::nanoseconds &)
{
  if (frame_num == 0 || frames == nullptr)
  {
    LOG_ERROR() << "frames pointer or frame_numis null";
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }
  frames->resize(frame_num);
  for (size_t i = 0; i < frames->size(); ++i)
  {
    for (int j = 0; j < CAN_FRAME_SIZE; ++j)
    {
      (*frames)[i].data[j] = static_cast<uint8_t>(j);
    }
    (*frames)[i].id = static_cast<uint32_t>(i);
    (*frames)[i].len = CAN_FRAME_SIZE;
    LOG_DEBUG() << (*frames)[i].CanFrameString() << "frame_num[" << i << "]";
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ++recv_counter_;
  return ErrorCode::OK;
}

NS_ZF::ErrorCode FakeCanClient::SetFilters(const std::string &)
{
  return NS_ZF::ErrorCode::OK;
}
NS_ZF::ErrorCode FakeCanClient::Wait(const std::chrono::nanoseconds)
{
  return NS_ZF::ErrorCode::OK;
}

std::string FakeCanClient::GetErrorString(const int32_t /*status*/)
{
  return "";
}

END_NS_ZF_DRIVER_CANBUS
