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
 * @brief Global defines.
 * @return
 */

#ifndef ZF_IO_GLOBAL_H
#define ZF_IO_GLOBAL_H

#include "zf_global/zf_global.h"

#define NS_IO io
#define BEGIN_NS_IO \
    namespace NS_IO \
    {
#define END_NS_IO }
#define NS_ZF_DRIVER_IO NS_ZF::NS_DRIVER::NS_IO              // zf::driver::io
#define BEGIN_NS_ZF_DRIVER_IO BEGIN_NS_ZF_DRIVER BEGIN_NS_IO // zf::driver::io
#define END_NS_ZF_DRIVER_IO END_NS_ZF_DRIVER END_NS_IO

#endif // !ZF_IO_GLOBAL_H
