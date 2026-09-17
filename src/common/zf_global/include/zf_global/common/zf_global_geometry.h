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
 * @brief Geomery structs defines.
 * @run
 */

#ifndef ZF_GLOBAL_COMMON_GEOMERY_H
#define ZF_GLOBAL_COMMON_GEOMERY_H

#include "zf_global/zf_global.h"

#define NS_GEOMERY            geometry
#define BEGIN_NS_GEOMERY      namespace NS_GEOMERY {
#define END_NS_GEOMERY        }
#define BEGIN_NS_ZF_GEOMERY   BEGIN_NS_ZF BEGIN_NS_GEOMERY
#define END_NS_ZF_GEOMERY     END_NS_GEOMERY END_NS_ZF
#define NS_ZF_GEOMERY         NS_ZF::NS_GEOMERY


BEGIN_NS_ZF_GEOMERY

struct Vector2D
{
  int x;
  int y;
};

struct Vector2DValue
{
  int x;
  int y;
  int value;
};

// Function to check for intersection between two line segments
inline Vector2D CheckIntersection(const Vector2D &p1, const Vector2D &p2, const Vector2D &p3, const Vector2D &p4) {
    double xdiff = p1.x - p2.x;
    double ydiff = p1.y - p2.y;
    double xdiff2 = p3.x - p4.x;
    double ydiff2 = p3.y - p4.y;

    double div = xdiff * ydiff2 - ydiff * xdiff2;
    if (div == 0) {
        return {0, 0};  // Lines do not intersect
    }
    double d[2] = {p1.x * p2.y - p1.y * p2.x, p3.x * p4.y - p3.y * p4.x};
    double x = (d[0] * xdiff2 - xdiff * d[1]) / div;
    double y = (d[0] * ydiff2 - ydiff * d[1]) / div;

    if (std::min(p1.x, p2.x) <= x && x <= std::max(p1.x, p2.x) &&
        std::min(p3.x, p4.x) <= x && x <= std::max(p3.x, p4.x)) {
        return {x, y};
    }
    return {0, 0};  // Intersection const Vector2D &is not within line segments
}


END_NS_ZF_GEOMERY

#endif // ZF_GLOBAL_COMMON_GEOMERY_H