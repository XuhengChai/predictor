#ifndef ZF_HAL_CAN_CLIENT_FAKE_H
#define ZF_HAL_CAN_CLIENT_FAKE_H

#include "zf_global/common/zf_global_macros.h"
#include "zf_hal_can_driver/client/can_client.h"

/**
 * @namespace zf::driver::canbus
 */
BEGIN_NS_ZF_DRIVER_CANBUS

/**
 * @class FakeCanClient
 * @brief The class which defines a fake CAN client which inherits CanClient.
 *        This fake CAN client is used for testing.
 */
class FakeCanClient : public CanClient
{
public:
  /**
   * @brief Initialize the fake CAN client by specified CAN card parameters.
   * @param parameter CAN card parameters to initialize the CAN client.
   * @return If the initialization is successful.
   */
  bool Init(const CANCardParameter &param) override;

  /**
   * @brief Destructor
   */
  virtual ~FakeCanClient() = default;

  /**
   * @brief Open the fake CAN client.
   * @return The status of the Open action which is defined by
   *         NS_ZF::ErrorCode.
   */
  NS_ZF::ErrorCode Open() override;

  /**
   * @brief Close the fake CAN client.
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
  NS_ZF::ErrorCode Receive(std::vector<CanFrame> *frames,
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
  int32_t send_counter_ = 0;
  int32_t recv_counter_ = 0;
  std::stringstream frame_info_;
};

END_NS_ZF_DRIVER_CANBUS

#endif // ZF_HAL_CAN_CLIENT_FAKE_H