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
 * @brief Global struct defines.
 * @return
 */

#ifndef ZF_IO_STRUCT_H
#define ZF_IO_STRUCT_H

#include "zf_global/in/zf_io_global.h"

BEGIN_NS_ZF_DRIVER_IO

// 结构体定义中的 __attribute__((__packed__)) ，它告诉编译器：字段内存无须对齐，避免造成空洞
typedef struct mac_frm_hdr
{
    u_char dest_addr[6]; // destination MAC address shall be defined first.
    u_char src_addr[6];
    short type; // 2 bytes; type, in network byte order
} __attribute__((packed)) MAC_FRM_HDR;

typedef struct ip_hdr
{ // header of IPV4
#ifdef __LITTLE_ENDIAN_BIFIELD
    u_char ip_len : 4, ip_ver : 4;
#else
    u_char ip_ver : 4, ip_len : 4;
#endif

    u_char ip_tos;
    u_short ip_total_len;
    u_short ip_id;
    u_short ip_flags;
    u_char ip_ttl;
    u_char ip_protocol;
    u_short ip_chksum;
    uint32_t ip_src;
    uint32_t ip_dest;
} __attribute__((packed)) IP_HDR;

typedef struct can_hdr
{
    // header of IPV4
#ifdef __LITTLE_ENDIAN_BIFIELD
    u_char ip_len : 4, ip_ver : 4;
#else
    u_char ip_ver : 4, ip_len : 4;
#endif
    u_char ip_tos;
    uint8_t canId;
    uint8_t config_data_flag;
    uint8_t canfd_flag;
    uint32_t msg_id;
    // uint8_t msg_id[4];
    uint8_t length;
    uint8_t data[64];
} __attribute__((packed)) CAN_HDR;

END_NS_ZF_DRIVER_IO

#endif // !ZF_IO_STRUCT_H
