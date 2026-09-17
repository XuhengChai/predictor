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
 * @brief Defines the CanId handler.
 * @run
 */


#ifndef ZF_HAL_CAN_ID_H
#define ZF_HAL_CAN_ID_H

#include "zf_hal_can_driver/hal_can_driver_global.h"

/**
 * @namespace NS_ZF::driver::canbus
 */
BEGIN_NS_ZF_DRIVER_CANBUS

enum class FrameType : uint32_t
{
  DATA,
  ERROR,
  REMOTE
  // SocketCan doesn't support Overload frame directly?
}; // enum class FrameType

/// A wrapper around can_id_t to make it a little more C++-y
/// WARNING: I'm assuming the 0th bit is the MSB aka the leftmost bit
class HAL_CAN_DRIVER_PUBLIC CanId
{
public:
  // Default constructor: standard data frame with id 0
  // CanId() = default;

  static uint32_t Identifier(const uint32_t &raw_id);
  /**
   * @brief GenerateCANID.
   * @param priority 3: 0C, 6: 18. The priority from 0 to 7. 
   * @param pgn string in Hex.
   * @param sourceId The sourceId.
   * @param destId The destId. default is 0 if destId is not specified.
   */
  static uint32_t GenerateCanId(uint8_t priority, const std::string &pgn, uint8_t sourceId, uint8_t destId = 0);
  static uint32_t GenerateCanId(uint32_t priority, const uint32_t &pgn, const std::string &ecuAddr);

  /**
   * @brief GenerateCANID.
   * @param raw_id must be raw data; Notice 18FF5EF0 is not raw data, 98FF5EF0 is.
   */
  CanId(const uint32_t raw_id); // explicit, used for recv msg; If donot know is_extended
  CanId(const uint32_t id, FrameType type, bool is_extended);// used for send msg; already know is_extended and type

  // Get the whole id value with 32 bit
  uint32_t IdRaw() const noexcept;
  // Get just the can_id with 29 bits
  uint32_t Id() const noexcept;

  // Check if frame is extended
  bool IsExtended() const noexcept;

  CanId &SetIDType(bool is_extended) noexcept;
  CanId &InitFrameType();
  CanId &SetFrameType(const FrameType type);

  /**
   * @brief Check frame type
   * \throw std::domain_error If bits are in an inconsistent state
   */
  FrameType GetFrameType() const;

private:
  // HAL_CAN_DRIVER_LOCAL
  /// Sets leading bits
  /// \throw std::domain_error If id would get truncated, 11 bits for Standard, 29 bits for Extended
  CanId &Init(const uint32_t id);
  /// Get just the can_id with 29 bits
  void Identifier() noexcept;
  FrameType m_type{};
  uint32_t m_idRaw{};
  uint32_t m_id{};
}; // class CanId

END_NS_ZF_DRIVER_CANBUS

#endif // ZF_HAL_CAN_ID_H