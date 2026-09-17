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
 * @brief Global micro defines.
 * @run
 */

#ifndef ZF_GLOBAL_COMMON_MACROS_H
#define ZF_GLOBAL_COMMON_MACROS_H

#include <memory>
#include <mutex>

#include "zf_global/zf_global.h"

BEGIN_NS_ZF

#if __GNUC__ >= 3
#define zf_likely(x) (__builtin_expect((x), 1))
#define zf_unlikely(x) (__builtin_expect((x), 0))
#else
#define zf_likely(x) (x)
#define zf_unlikely(x) (x)
#endif

// There must be many copy-paste versions of these macros which are same
// things, undefine them to avoid conflict.
#undef DISALLOW_COPY_AND_ASSIGN

#define DISALLOW_COPY_AND_ASSIGN(classname) \
  classname(const classname &) = delete;    \
  classname &operator=(const classname &) = delete;

#define DECLARE_SINGLETON(classname)                                      \
 public:                                                                  \
  static classname &Instance(bool create_if_needed = true) {              \
    static std::unique_ptr<classname> instance = nullptr;                 \
    if (!instance && create_if_needed) {                                  \
      static std::once_flag flag;                                         \
      std::call_once(flag,                                                \
                    [&] { instance.reset(new classname); });  \
    }                                                                     \
    return *instance;                                                     \
  }                                                                       \
                                                                          \
 private:                                                                 \
  classname();                                                            \
  DISALLOW_COPY_AND_ASSIGN(classname)

// static classname *instance = nullptr;                                 
//  [&] { instance = new (std::nothrow) classname(); }); 

#define enum2str(val) #val// string str = enum2str(ONCE)

END_NS_ZF

#endif // !ZF_GLOBAL_COMMON_MACROS_H