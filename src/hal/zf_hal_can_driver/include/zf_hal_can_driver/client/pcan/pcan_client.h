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
 * @brief Defines the socket CanClient interface.
 * @run
 */

#ifndef ZF_HAL_PCAN_CLIENT_H
#define ZF_HAL_PCAN_CLIENT_H

#include <unistd.h>

// #include <net/if.h>
// #include <sys/ioctl.h>
// #include <sys/socket.h>
// #include <sys/types.h>

// #include <linux/can.h>
// #include <linux/can/raw.h>
#include "PCANBasic.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "zf_hal_can_driver/client/can_client.h"
#include "zf_global/common/zf_global_macros.h"
#include "zf_global/util/factory.h"

/**
 * @namespace zf::driver::canbus
 */
BEGIN_NS_ZF_DRIVER_CANBUS

class PCanClient : public CanClient
{
public:
  /**
   * @brief Constructor and Destructor
   */
  PCanClient();
  virtual ~PCanClient();

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
  bool LookUpChannels();
  bool CheckForLibrary();
  void InitBaud();
  void ShowStatus(TPCANStatus status);
  fd_set SingleSet(int32_t file_descriptor) noexcept;
  TPCANHandle m_iPcanHandler = 0;
  bool m_fdMode;
  TPCANBaudrate m_iBaudrate;
	/// <summary>
	/// Sets the bitrate for CAN FD devices.
	/// Example - Bitrate Nom: 1Mbit/s Data: 2Mbit/s:
	///   "f_clock_mhz=20, nom_brp=5, nom_tseg1=2, nom_tseg2=1, nom_sjw=1, data_brp=2, data_tseg1=3, data_tseg2=1, data_sjw=1"
	/// </summary>
	TPCANBitrateFD m_sBitrateFD;
  // CANCardParameter m_canParam;
  std::vector<TPCANHandle> m_vecHandlers;
  TPCANMsg send_frames_[MAX_CAN_SEND_FRAME_LEN];
  // canfd_frame send_frames_fd[MAX_CAN_SEND_FRAME_LEN];// to be test if should be used?
  TPCANMsg recv_frames_[MAX_CAN_RECV_FRAME_LEN];
};

END_NS_ZF_DRIVER_CANBUS

#endif // ZF_HAL_PCAN_CLIENT_H