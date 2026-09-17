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
 * @brief Error code defines.
 * @run
 */

#ifndef ZF_GLOBAL_ERROR_CODE_H
#define ZF_GLOBAL_ERROR_CODE_H

#include "zf_global/zf_global.h"

BEGIN_NS_ZF

enum class ErrorCode: uint32_t
{
    // No error, returns on success.
    OK = 0,
    INIT = 10,
    READY = 30,
    DOWNGRADE = 50,
    ERROR = 60,

    // Control module error codes start from here.
    CONTROL_ERROR = 0x60001000,
    CONTROL_INIT_ERROR = 0x60001001,
    CONTROL_COMPUTE_ERROR = 0x60001002,
    CONTROL_ESTOP_ERROR = 0x60001003,
    PERFECT_CONTROL_ERROR = 0x60001004,

    // Canbus module error codes start from here.
    CANBUS_ERROR = 0x60002000,
    CAN_CLIENT_ERROR_BASE = 0x60002100,
    CAN_CLIENT_ERROR_OPEN_DEVICE_FAILED = 0x60002101,
    CAN_CLIENT_ERROR_FRAME_NUM = 0x60002102,
    CAN_CLIENT_ERROR_SEND_FAILED = 0x60002103,
    CAN_CLIENT_ERROR_RECV_FAILED = 0x60002104,
    CAN_CLIENT_ERROR_TIMEOUT = 0x60002105,

    CAN_PARSE_INVALID_NAME = 0x60002150,
    CAN_PARSE_CHECK_FAILED = 0x30002151,
    CAN_PARSE_OUT_OF_BOUND = 0x30002152,
    CAN_PARSE_IGNORE = 0x30002153,

    // Localization module error codes start from here.
    LOCALIZATION_ERROR = 0x60003000,
    LOCALIZATION_ERROR_MSG = 0x60003100,
    LOCALIZATION_ERROR_LIDAR = 0x60003200,
    LOCALIZATION_ERROR_INTEG = 0x60003300,
    LOCALIZATION_ERROR_GNSS = 0x60003400,

    // Perception module error codes start from here.
    PERCEPTION_ERROR = 0x60004000,
    PERCEPTION_ERROR_TF = 0x60004001,
    PERCEPTION_ERROR_PROCESS = 0x60004002,
    PERCEPTION_FATAL = 0x60004003,
    PERCEPTION_ERROR_NONE = 0x60004004,
    PERCEPTION_ERROR_UNKNOWN = 0x60004005,

    // Prediction module error codes start from here.
    PREDICTION_ERROR = 0x60005000,

    // Planning module error codes start from here
    PLANNING_ERROR = 0x60006000,
    PLANNING_ERROR_NOT_READY = 0x60006001,

    // HDMap module error codes start from here
    HDMAP_DATA_ERROR = 0x60007000,

    // Routing module error codes
    ROUTING_ERROR = 0x60008000,
    ROUTING_ERROR_REQUEST = 0x60008001,
    ROUTING_ERROR_RESPONSE = 0x60008002,
    ROUTING_ERROR_NOT_READY = 0x60008003,

    // Indicates an input has been exhausted.
    END_OF_INPUT = 0x60009000,

    // HTTP request error codes.
    HTTP_LOGIC_ERROR = 0x60010000,
    HTTP_RUNTIME_ERROR = 0x60010001,

    // Relative Map error codes.
    RELATIVE_MAP_ERROR = 0x60011000, // general relative map error code
    RELATIVE_MAP_NOT_READY = 0x60011001,

    // Driver error codes.
    DRIVER_ERROR_GNSS = 0x60012000,
    DRIVER_ERROR_VELODYNE = 0x60013000,

    // Storytelling error codes.
    STORYTELLING_ERROR = 0x60014000,

    //
    SOCKET_ERROR = 0x60014000,

};

enum class ENodeStatus: uint32_t
{
    // RUNNING = 0x60000,
    INIT = 10,
    READY = 30,
    DOWNGRADE = 50,
    ERROR = 60
};

enum ECanMsgStatus: uint8_t
{
    OK = 48,
    ERROR = 49
};

END_NS_ZF

#endif // !ZF_GLOBAL_ERROR_CODE_H
