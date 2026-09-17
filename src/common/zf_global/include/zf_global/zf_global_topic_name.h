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
 * @brief Global ros topic names defines.
 * @run
 */

#ifndef ZF_GLOBAL_TOPIC_NAME_H
#define ZF_GLOBAL_TOPIC_NAME_H

#include "zf_global.h"

BEGIN_NS_ZF

static const char gk_camImageRaw[] = "/camera/image_raw";
static const char gk_camImageCV[] = "/camera/image_cv";  //.jpg

static const char gk_camImageConvert[] = "/image_converter/img";  //.jpg

static const char gk_halNodeStatusCanRaw[] = "/node_status/can_raw";  //
static const char gk_halNodeStatusCanMsg[] = "/node_status/can_msg";  //

static const char gk_halFromCanRawBus[] = "/hal/from_can_raw_bus";  //
static const char gk_halToCanRawBus[] = "/hal/to_can_raw_bus";      //
static const char gk_halToUi[] = "/hal/to_ui";                      //
static const char gk_halRadarStatusToUi[] = "/hal/to_ui_radar_status";                      //

static const char gk_halFromCanMsgData[] = "/hal/from_can_msg_data";  //
static const char gk_halToCanMsgData[] = "/hal/to_can_msg_data";      //
static const char gk_halToCanSigData[] =
    "/hal/to_can_sig_data";  // other signal will change to zero, deprecated

static const char gk_halFromPackedRadar[] = "/hal/from_packed_radar_data";  //

static const char gk_halFromSyncData[] = "/hal/from_sync_datas";  //
static const char gk_halSensorObjs[] = "/hal/sensor_objs";        //
static const char gk_halNodeStatusSync[] = "/node_status/sync";   //

static const char gk_uiSystemState[] = "/system_state";  //
static const char gk_uiDataSystemState[] = "/data_sys_manager/sys_state";  //
static const char gk_uiBevData[] = "/hmi_bev_data";      //

static const char gk_egoInfoSideCam[] = "/ego_info_side_cam";  //
static const char gk_egoInfoFrontCam[] = "/ego_info_front_cam";  //

//channel name

static const char gk_channelCamToUi[] = "img_to_ui";
static const char gk_channelCamToDet[] = "img_to_dect_track";
static const char gk_channelDetToUi[] = "img_dect_to_ui";

END_NS_ZF

#endif  // !ZF_GLOBAL_TOPIC_NAME_H
