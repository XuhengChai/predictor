#ifndef ZF_HAL_CAN_CLIENT_ETHERNET_H
#define ZF_HAL_CAN_CLIENT_ETHERNET_H

#include <sstream>
#include <iomanip>
#include <memory>
#include <thread>

#include "zf_hal_can_driver/client/can_client.h"
#include "zf_global/in/zf_framework_global.h"
#include "zf_global/in/zf_io_struct.h"
#include "zf_global/util/ring_buffer.h"

BEGIN_NS_ZF_FRAMEWORK  // zf::framework
    class MsgBase;

template <typename M>
class Transmitter;

template <typename M>
class Receiver;

using TransmitterPtr = std::shared_ptr<Transmitter<MsgBase>>;
using ReceiverPtr = std::shared_ptr<Receiver<MsgBase>>;

END_NS_ZF_FRAMEWORK

/**
  * @namespace zf::driver::canbus
  */
BEGIN_NS_ZF_DRIVER_CANBUS

class EthernetClient : public CanClient
{
public:
  using CANRingBuffer = RingBuffer<CanFrame>;
  EthernetClient(/* args */);
  ~EthernetClient();
  // void Init(const char *iface = "eth4");

  /**
   * @brief Initialize the CAN client by specified CAN card parameters.
   * @param parameter CAN card parameters to initialize the CAN client.
   * @return If the initialization is successful.
   */
  bool Init(const CANCardParameter &parameter) override;

  /**
   * @brief Open the CAN client.
   * @return The status of the Open action which is defined by
   *         NS_ZF::ErrorCode.
   */
  NS_ZF::ErrorCode Open() override;

  /**
   * @brief Close the CAN client.
   */
  void Close() override;

  /**
   * @brief Send messages
   * @param frames The messages to send.
   * @param frame_num The amount of messages to send.
   * @return The status of the sending action which is defined by
   *         NS_ZF::ErrorCode.
   */
  NS_ZF::ErrorCode Send(const std::vector<CanFrame> &frames,
                        const int32_t &frame_num,
                        const std::chrono::nanoseconds &timeout = std::chrono::nanoseconds::zero()) override;

  /**
   * @brief Receive messages
   * @param frames The messages to receive.
   * @param frame_num The amount of messages to receive.
   * @return The status of the receiving action which is defined by
   *         NS_ZF::ErrorCode.
   */
  NS_ZF::ErrorCode Receive(std::vector<CanFrame> *const frames,
                           const int32_t &frame_num,
                           const std::chrono::nanoseconds &timeout = std::chrono::nanoseconds::zero()) override;

  NS_ZF::ErrorCode SetFilters(const std::string &str) override;

  NS_ZF::ErrorCode Wait(const std::chrono::nanoseconds timeout) override;

  /**
   * @brief Get the error string.
   * @param status The status to get the error string.
   */
  std::string GetErrorString(const int32_t status) override;

private:
  void CanFrame2CAN_HDR(const CanFrame &frame, NS_ZF_DRIVER_IO::CAN_HDR &header);
  void CAN_HDR2CanFrame(const NS_ZF_DRIVER_IO::CAN_HDR *header, CanFrame &frame);
  bool ParserBuffer(char *, CanFrame &frame);
  void InitTransmit(){};
  void InitReceiver();
  void LoopListen();
  NS_ZF_FRAMEWORK::TransmitterPtr m_transmitterEthCAN;
  NS_ZF_FRAMEWORK::ReceiverPtr m_recvEthCAN;
  NS_ZF_DRIVER_IO::MAC_FRM_HDR m_frameMacSender;
  // NS_ZF_FRAMEWORK::SegmentPtr m_segment;
  std::unique_ptr<CANRingBuffer> m_canBuffer;
  std::atomic_bool m_bIsShutDown = {};
  std::thread m_threadListen;
  time_t m_lLastSec;
  int m_cnt = {};
};

END_NS_ZF_DRIVER_CANBUS

#endif // ZF_HAL_CAN_CLIENT_ETHERNET_H
