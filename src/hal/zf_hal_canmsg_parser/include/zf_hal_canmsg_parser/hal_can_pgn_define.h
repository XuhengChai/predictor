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
 * @brief Defines the can info interface for ros node and can parser pack.
 * @run
 */

#ifndef ZF_HAL_CANMSG_PGN_DEFINE_H_
#define ZF_HAL_CANMSG_PGN_DEFINE_H_

#include <string>
#include "zf_hal_can_driver/hal_can_driver_global.h"

// /**
//  * @namespace NS_ZF::driver::canbus
//  */
BEGIN_NS_ZF_DRIVER_CANBUS

static const char gk_pgnTSC1[] = "0000";
static const char gk_pgnXBR[] = "0400"; // 200ms
static const char gk_pgnAEBS2[] = "0B00";
static const char gk_pgnEBC1_EBS[] = "F001"; // 20ms
static const char gk_pgnEBC2[] = "FEBF";
static const char gk_pgnETC2[] = "F005"; // 100ms
static const char gk_pgnVDC2[] = "F009"; // 10ms
static const char gk_pgnEEC1[] = "F004"; // motor
static const char gk_pgnEEC2[] = "F003"; // motor 50ms
static const char gk_pgnEEC3[] = "FEDF"; // 250ms

static const char gk_pgnCCVS1[] = "FEF1";
static const char gk_pgnEBC2_0B[] = "FEBF";
static const char gk_pgnEBC5_0B[] = "FDC4";
static const char gk_pgnERC1_29[] = "F000";
static const char gk_pgnERC1_0F[] = "F000";
static const char gk_pgnERC1_10[] = "F000";
static const char gk_pgnTD_EE[] = "FEE6";
static const char gk_pgnTCO1[] = "FE6C"; // 50ms
static const char gk_pgnVDC1[] = "FE4F"; // 18
static const char gk_pgnETC1[] = "F002"; // 10ms
static const char gk_pgnETC2_03[] = "F005";
static const char gk_pgnVDC2_0B[] = "F009"; // 10ms
static const char gk_pgnHRW_0B[] = "FE6E";  // 20ms
static const char gk_pgnVDHR_EE[] = "FEC1"; // EE
static const char gk_pgnCCSS[] = "FEED";
static const char gk_pgnOEL_32[] = "FDCC"; // 32 1000ms
static const char gk_pgnOWW_32[] = "FDCD"; // 32
static const char gk_pgnCVW_0B[] = "FE70"; // 1000ms
static const char gk_pgnCVW_03[] = "FE70";

static const char gk_keyTPDTpgn[] = "ZF_TPDT_pgn";

enum ECanId : uint32_t {
    EBC5_EBS = 0x18FDC40B,
    EC1 = 0x18FEE300,
    EC00 = 0x18ECFF00,
    EB00 = 0x18EBFF00,
    EEC2 = 0x0CF00300,
    EEC3 = 0x18FEDF00,
    ETC1 = 0x0CF00203,
    ETC2 = 0x18F00503,
    HRW = 0x08FE6E0B,
    CVW_AMT = 0x18FE7003,
    CVW_EBS = 0x18FE700B,
    VDC2 = 0x18F0090B,
    TCO1 = 0x0CFE6CEE,
    XBR_AEBS = 0x0C040BA0,
    LD = 0x18FE4021,
    OEL = 0x18FDCC21,
    EBC1_EBS = 0x18F0010B,
    DM1_TCM = 0x18FECA03,
    DM1_ENG = 0x18FECA00,
    DM1_EBS = 0x18FECA0B,
    StwError2Orin = 0x18FF3181,
    SRR_S5_C0 = 0x18FF74C0,
    SRR_S1_B0 = 0x18FF72B0
};

struct SignalAlias
{
    std::string alias;
    ECanId id;
    std::string sigName;
};

const std::vector<SignalAlias> gk_egoSignals = {
    {"xbr_deceleration_limit", ECanId::EBC5_EBS, "XBRAccelerationLimit"},
    {"reference_torque", ECanId::EC1, "EngReferenceTorque"},
    {"max_available_torque", ECanId::EEC2, "ActMaxAvailEngPercentTorque"},
    {"accel_pedal_position", ECanId::EEC2, "AccelPedalPos1"},
    {"friction_torque", ECanId::EEC3, "NominalFrictionPercentTorque"},
    {"tsmo_rpm", ECanId::ETC1, "TransOutputShaftSpeed"},
    {"tsmi_rpm", ECanId::ETC1, "TransInputShaftSpeed"},
    {"shift_process", ECanId::ETC1, "TransShiftInProcess"},
    {"gear_ratio", ECanId::ETC2, "TransActualGearRatio"},
    {"current_gear", ECanId::ETC2, "TransCurrentGear"},
    {"hrw_speed_fl", ECanId::HRW, "FrontAxleLeftWheelSpeed"},
    {"hrw_speed_fr", ECanId::HRW, "FrontAxleRightWheelSpeed"},
    {"weight_amt", ECanId::CVW_AMT, "GrossCombinationVehicleWeight"},
    {"weight_ebs", ECanId::CVW_EBS, "GrossCombinationVehicleWeight"},
    {"tractor_yawrate", ECanId::VDC2, "YawRate"},
    {"wheel_angle", ECanId::VDC2, "SteerWheelAngle"},
    {"acceleration", ECanId::VDC2, "LongitudinalAcceleration"},
    {"tach_speed", ECanId::TCO1, "TachographVehicleSpeed"},
    {"turning_light_right", ECanId::LD, "RightTurnSignalLightsData"},
    {"turning_light_switch", ECanId::OEL, "TurnSignalSwitch"},
    {"brake_pedal_position", ECanId::EBC1_EBS, "BrakePedalPos"},
    {"pri_xbr_aeb", ECanId::XBR_AEBS, "XBRPriority"},
    {"amber_lamp_ebs", ECanId::DM1_EBS, "AmberWarningLampStatus"},
    {"red_lamp_ebs", ECanId::DM1_EBS, "RedStopLampState"},
    {"amber_lamp_eng", ECanId::DM1_ENG, "AmberWarningLampStatus"},
    {"red_lamp_eng", ECanId::DM1_ENG, "RedStopLampState"},
    {"amber_lamp_tcm", ECanId::DM1_TCM, "AmberWarningLampStatus"},
    {"red_lamp_tcm", ECanId::DM1_TCM, "RedStopLampState"}
};

END_NS_ZF_DRIVER_CANBUS

#endif /* ZF_HAL_CANMSG_PGN_DEFINE_H_ */