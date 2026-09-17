/******************************************************************************
 * Copyright 2023 ZF. All Rights Reserved.
 * Author:
 *        xuheng.chai@zf.com
 *
 * Licensed under the Apache License, VersFRAMEWORKn 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITFRAMEWORKNS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissFRAMEWORKns and
 * limitatFRAMEWORKns under the License.
 *****************************************************************************/
/**
 * @file
 * @brief Global defines.
 * @return
 */

#include "zf_global/zf_global.h"

#ifndef ZF_FRAMEWORK_GLOBAL_H
#define ZF_FRAMEWORK_GLOBAL_H

#define NS_FRAMEWORK                            framework
#define BEGIN_NS_FRAMEWORK                      namespace NS_FRAMEWORK{
#define END_NS_FRAMEWORK                        }
#define NS_ZF_FRAMEWORK                         NS_ZF::NS_FRAMEWORK //zf::driver::FRAMEWORK
#define BEGIN_NS_ZF_FRAMEWORK                   BEGIN_NS_ZF BEGIN_NS_FRAMEWORK //zf::driver::FRAMEWORK
#define END_NS_ZF_FRAMEWORK                     END_NS_ZF END_NS_FRAMEWORK

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FRAMEWORK_TRANS_EXPORT __attribute__ ((dllexport))
    #define FRAMEWORK_TRANS_IMPORT __attribute__ ((dllimport))
  #else
    #define FRAMEWORK_TRANS_EXPORT __declspec(dllexport)
    #define FRAMEWORK_TRANS_IMPORT __declspec(dllimport)
  #endif
  #ifdef FRAMEWORK_TRANS_BUILDING_LIBRARY
    #define FRAMEWORK_TRANS_PUBLIC FRAMEWORK_TRANS_EXPORT
  #else
    #define FRAMEWORK_TRANS_PUBLIC FRAMEWORK_TRANS_IMPORT
  #endif
  #define FRAMEWORK_TRANS_PUBLIC_TYPE FRAMEWORK_TRANS_PUBLIC
  #define FRAMEWORK_TRANS_LOCAL
#else
  #define FRAMEWORK_TRANS_EXPORT __attribute__ ((visibility("default")))
  #define FRAMEWORK_TRANS_IMPORT
  #if __GNUC__ >= 4
    #define FRAMEWORK_TRANS_PUBLIC __attribute__ ((visibility("default")))
    #define FRAMEWORK_TRANS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FRAMEWORK_TRANS_PUBLIC
    #define FRAMEWORK_TRANS_LOCAL
  #endif
  #define FRAMEWORK_TRANS_PUBLIC_TYPE
#endif

#if !defined(RETURN_VAL_IF_NULL)
#define RETURN_VAL_IF_NULL(ptr, val) \
  if (ptr == nullptr) {              \
    LOG_WARN() << #ptr << " is nullptr."; \
    return val;                      \
  }
#endif


static const char gk_hostIp[] = "127.0.0.1";

#define SHARED_MEMORY_KEY_TC397 "shm_key_tc397"
#define BUFFER_MAX 2048
#define BUFFER_CAN_DATA_397 88

#endif // !ZF_FRAMEWORK_GLOBAL_H
