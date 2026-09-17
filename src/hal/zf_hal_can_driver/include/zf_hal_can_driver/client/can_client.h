/******************************************************************************
 * Copyright 2023 ZF. All Rights Reserved.
 * Author:
 *        xuheng.chai@zf.com
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
/**
 * @file
 * @brief Defines the CanClient interface.
 * @run
 */

#ifndef ZF_HAL_CAN_CLIENT_H
#define ZF_HAL_CAN_CLIENT_H

#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include "zf_hal_can_driver/client/can_frame.h"
#include "zf_hal_can_driver/hal_can_driver_global.h"

/**
 * @namespace NS_ZF::driver::canbus
 */
BEGIN_NS_ZF_DRIVER_CANBUS

class CanFilterList;

/**
 * @class CanClient
 * @brief The class which defines the CAN client to send and receive message.
 */
class CanClient
{
public:
  /**
   * @brief Constructor
   */
  CanClient();

  /**
   * @brief Destructor
   */
  virtual ~CanClient();
  CanClient(CanClient &&rhs); // 同析构函数，仅声明
  CanClient &operator=(CanClient &&rhs);
  CANCardParameter GetParameter(){return m_canParam;};

  /**
   * @brief Initialize the CAN client by specified CAN card parameters.
   * @param parameter CAN card parameters to initialize the CAN client.
   * @return If the initialization is successful.
   */
  virtual bool Init(const CANCardParameter &parameter) = 0;

  /**
   * @brief Open the CAN client.
   * @return The status of the Open action which is defined by ErrorCode.
   */
  virtual NS_ZF::ErrorCode Open() = 0;

  /**
   * @brief Close the CAN client.
   */
  virtual void Close() = 0;

  /**
   * @brief Send messages
   * @param frames The messages to send.
   * @param frame_num The amount of messages to send.
   * @return The status of the sending action which is defined by ErrorCode.
   */
  virtual NS_ZF::ErrorCode Send(const std::vector<CanFrame> &frames,
                                const int32_t &frame_num,
                                const std::chrono::nanoseconds &timeout = std::chrono::nanoseconds::zero()) = 0;

  /**
   * @brief Send a single message.
   * @param frames A single-element vector containing only one message.
   * @return The status of the sending single message action which is defined by
   *         NS_ZF::ErrorCode.
   */
  virtual NS_ZF::ErrorCode SendSingleFrame(
      const std::vector<CanFrame> &frames,
      const std::chrono::nanoseconds &timeout = std::chrono::nanoseconds::zero())
  {
    CHECK_EQ(frames.size(), 1U) << "frames size not equal to 1, actual frame size :" << frames.size();
    return Send(frames, 1, timeout);
  }

  /**
   * @brief Receive messages
   * @param frames The messages to receive.
   * @param frame_num The amount of messages to receive.
   * @return The status of the receiving action which is defined by
   *         NS_ZF::ErrorCode.
   */
  virtual NS_ZF::ErrorCode Receive(std::vector<CanFrame> *const frames,
                                   const int32_t &frame_num,
                                   const std::chrono::nanoseconds &timeout = std::chrono::nanoseconds::zero()) = 0;

  virtual NS_ZF::ErrorCode SetFilters(const std::string &str) = 0;

  virtual NS_ZF::ErrorCode Wait(const std::chrono::nanoseconds timeout) = 0;

  /**
   * @brief Get the error string.
   * @param status The status to get the error string.
   */
  virtual std::string GetErrorString(const int32_t status) = 0;

protected:
  /// The CAN client is Opened.
  bool m_bIsOpened = {};
  CANCardParameter m_canParam;
  std::unique_ptr<CanFilterList> m_filter;
  // CanFilterList *m_filter;
};

END_NS_ZF_DRIVER_CANBUS

#endif // ZF_HAL_DRIVER_CAMERA_H