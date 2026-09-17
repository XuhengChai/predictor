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
 * @brief Create socket can or fake can.
 * @run
 */

#ifndef ZF_HAL_CAN_CLIENT_FACTORY_H
#define ZF_HAL_CAN_CLIENT_FACTORY_H

#include "zf_hal_can_driver/client/can_client.h"
#include "zf_global/common/zf_global_macros.h"
#include "zf_global/util/factory.h"

/**
 * @namespace zf::driver::canbus
 */
BEGIN_NS_ZF_DRIVER_CANBUS

class CanClientFactory
    : public NS_ZF::Factory<CANCardParameter::CANCardBrand, CanClient>
{
public:
  /**
   * @brief Register the CAN clients of all brands. This function call the
   *        Function apollo::common::util::Factory::Register() for all of the
   *        CAN clients.
   */
  void RegisterCanClients();

  /**
   * @brief Create a pointer to a specified brand of CAN client. The brand is
   *        set in the parameter.
   * @param parameter The parameter to create the CAN client.
   * @return A pointer to the created CAN client.
   */
  std::unique_ptr<CanClient> CreateCANClient(const CANCardParameter &parameter);

private:
  DECLARE_SINGLETON(CanClientFactory)
};

END_NS_ZF_DRIVER_CANBUS

#endif // ZF_HAL_CAN_CLIENT_FACTORY_H