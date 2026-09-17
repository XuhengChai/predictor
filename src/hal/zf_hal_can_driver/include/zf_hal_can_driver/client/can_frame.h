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
 * @brief Defines the CanFrame struct and can parameters.
 * @run
 */

#ifndef ZF_HAL_CAN_FRAME_H
#define ZF_HAL_CAN_FRAME_H

#include <cstdint>
#include <cstring>
#include <sstream>
#include <vector>
#include <chrono>

#include "zf_hal_can_driver/byte.h"

/**
 * @namespace zf::driver::canbus
 */
BEGIN_NS_ZF_DRIVER_CANBUS

// const int CAN_RESULT_SUCC = 0;
// const int CAN_ERROR_BASE = 2000;
// const int CAN_ERROR_OPEN_DEVICE_FAILED = CAN_ERROR_BASE + 1;
// const int CAN_ERROR_FRAME_NUM = CAN_ERROR_BASE + 2;
// const int CAN_ERROR_SEND_FAILED = CAN_ERROR_BASE + 3;
// const int CAN_ERROR_RECV_FAILED = CAN_ERROR_BASE + 4;
// const int CAN_CLIENT_ERROR_TIMEOUT = CAN_ERROR_BASE + 5,

const int32_t CAN_FRAME_SIZE = 8;
const int32_t MAX_CAN_SEND_FRAME_LEN = 1;
const int32_t MAX_CAN_RECV_FRAME_LEN = 10;
const int32_t CANBUS_MESSAGE_LENGTH = 8;     // according to ISO-11891-1
const int32_t CANBUS_MESSAGE_LENGTH_FD = 64; // according to ISO-11891-1

struct CANCardParameter
{
  enum CANCardBrand
  {
    FAKE_CAN = 0,
    SOCKET_CAN = 1,
    ETHERNET_CAN = 2,
    PCAN = 3,
  };

  enum CANCardType
  {
    PCI_CARD = 0,
    USB_CARD = 1,
  };

  enum CANChannelId
  {
    CHANNEL_ID_ZERO = 0,
    CHANNEL_ID_ONE = 1,
    CHANNEL_ID_TWO = 2,
    CHANNEL_ID_THREE = 3,
    CHANNEL_ID_FOUR = 4,
    CHANNEL_ID_FIVE = 5,
    CHANNEL_ID_SIX = 6,
    CHANNEL_ID_SEVEN = 7,
    CHANNEL_ID_MAX = 8,
  };

  enum EXTERNALCANChannelId
  {
    EXTERNAL_ID_0 = 1,
    EXTERNAL_ID_1 = 2,
    EXTERNAL_ID_2 = 3,
    EXTERNAL_ID_3 = 4,
    EXTERNAL_ID_10 = 5,
    EXTERNAL_ID_11 = 6,
    EXTERNAL_MCU_CAN = 7,
    EXTERNAL_ID_13 = 8,
  };

  enum CANInterface
  {
    NATIVE = 0,
    VIRTUAL = 1,
    SLCAN = 2,
  };

  enum CANBaudrate
  {
    BCAN_BAUDRATE_NUM = 0,
    BCAN_BAUDRATE_100K = 100,
    BCAN_BAUDRATE_250K = 250,
    BCAN_BAUDRATE_500K = 500,
    BCAN_BAUDRATE_1M = 1000,
  };

  enum CANDescription
  {
    CAN_DES_VEHICLE = 0,
    CAN_DES_5G4T = CAN_DES_VEHICLE + 1, // 1
    CAN_DES_IPM = CAN_DES_5G4T + 1,     // 2
    CAN_DES_AC1000T = CAN_DES_VEHICLE,  // 0
  };

  CANCardBrand brand = SOCKET_CAN;
  CANChannelId channelID = CHANNEL_ID_ZERO;
  CANBaudrate baudrate = BCAN_BAUDRATE_500K;
  CANInterface interface = NATIVE;
  timeval timestamp{0, 200'000U};
  bool enableFD = false;
  uint8_t portsNum = CHANNEL_ID_SEVEN + 1;
  std::string deviceName = "";
  void operator=(const CANCardParameter &right)
  {
    this->brand = right.brand;
    this->channelID = right.channelID;
    this->baudrate = right.baudrate;
    this->interface = right.interface;
    this->timestamp = right.timestamp;
    this->enableFD = right.enableFD;
    this->deviceName = right.deviceName;
  }
  std::string DebugString() const
  {
    std::stringstream ss;
    ss << "with brand is " << brand << ", channelID: " << channelID << ", baudrate: "
       << baudrate << ", interface: " << interface << ", enableFD: " << enableFD;
    return ss.str();
  }
};

/**
 * @class CanFrame
 * @brief The class which defines the information to send and receive.
 */
struct CanFrame
{
  uint8_t channelID = 0;
  /// Message id
  uint32_t id;
  /// Message length
  uint8_t len;
  /// Message content
  uint8_t data[64] = {};
  /// Time stamp   long tv_sec;	 long tv_usec	/* Microseconds.  */
  struct timeval timestamp;

  /**
   * @brief Constructor
   */
  CanFrame() : id(0), len(0), timestamp{0, 0}
  {
    // std::memset(data, 0, sizeof(data));
  }

  uint32_t IdentifierID() const;
  struct timeval to_timeval(const std::chrono::nanoseconds timeout) noexcept;
  /// Convert timeval to time in microseconds
  uint64_t from_timeval(const struct timeval tv) noexcept;

  /**
   * @brief CanFrame string including essential information about the message.
   * @return The info string.
   */
  std::string CanFrameString() const
  {
    std::stringstream output_stream("");
    output_stream << "[" << timestamp.tv_sec << "." << std::setw(6) << std::setfill('0') << timestamp.tv_usec
                  << "], id: " << Byte::PGNFromCanId(IdentifierID())
                  // << ", " << Byte::Int2Hex(IdentifierID())
                  << ", " << id
                  << ", " << IdentifierID()
                  << ", data:";
    // << ", len:" << static_cast<int>(len) << ", data:";
    for (uint8_t i = 0; i < len; ++i)
    {
      output_stream << Byte::Int2Hex(data[i]) << " ";
    }
    output_stream << ",";
    return output_stream.str();
  }
};

END_NS_ZF_DRIVER_CANBUS

#endif // ZF_HAL_CAN_FRAME_H