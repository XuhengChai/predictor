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
 * WITHOUT WARRANTIES OR CONDITUINS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
/**
 * @file
 * @brief Global defines.
 * @return
 */

#ifndef ZF_UI_GLOBAL_H
#define ZF_UI_GLOBAL_H

#include "zf_global/zf_global.h"

#define NS_UI ui
#define BEGIN_NS_UI \
    namespace NS_UI \
    {
#define END_NS_UI }
#define BEGIN_NS_ZF_UI BEGIN_NS_ZF BEGIN_NS_UI // zf::driver::ui
#define END_NS_ZF_UI END_NS_ZF END_NS_UI

#endif // !ZF_UI_GLOBAL_H
