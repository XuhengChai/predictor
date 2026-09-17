
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
 * @brief Defines the hal datas sync ros node.
 * @return
 */

// #ifndef ZF_HAL_APPROXIMATE_TIME_SYNC_H_
// #define ZF_HAL_APPROXIMATE_TIME_SYNC_H_
#ifndef ZF_HAL_EXACT_TIME_SYNC_H_
#define ZF_HAL_EXACT_TIME_SYNC_H_

#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <type_traits>

#include "zf_global/in/hal_sync_global.h"
#include "time_sync.h"

// /**
//  * @namespace NS_ZF::driver::synchronization
//  */
BEGIN_NS_ZF_HAL_SYNC

template<typename M0, typename M1, typename M2 = NullType, typename M3 = NullType, typename M4 = NullType,
         typename M5 = NullType, typename M6 = NullType, typename M7 = NullType, typename M8 = NullType>
class ApproximateTimeSync: public TimeSynchronizer<M0, M1, M2, M3, M4, M5, M6, M7, M8>
{
private:
    /* data */
public:
    ApproximateTimeSync();
    ~ApproximateTimeSync();

    virtual void TryToProcess(const double time) override
    {
        if (fabs(MinTime() - time) > 1e-7)
        {
            return;
        }
        M0PairPtr m0 = nullptr;
        M1PairPtr m1 = nullptr;
        M2PairPtr m2 = nullptr;
        M3PairPtr m3 = nullptr;
        for (size_t i = 0; i < m_sizeClass; ++i)
        {
            if (i == 0)
            {
                m0 = ExactTimeValue<M0Pair>(time, m_vDataQueue[0]);
            }
            if (i == 1)
            {
                m1 = ExactTimeValue<M1Pair>(time, m_vDataQueue[1]);
            }
            if (i == 2)
            {
                m2 = ExactTimeValue<M2Pair>(time, m_vDataQueue[2]);
            }
            if (i == 3)
            {
                m3 = ExactTimeValue<M3Pair>(time, m_vDataQueue[3]);
            }
        }
        const bool m0_ok = m0 != nullptr;
        const bool m1_ok = m1 != nullptr;
        const bool m2_ok = m2 != nullptr;
        const bool m3_ok = m3 != nullptr;

        if (m0_ok && m1_ok && m_sizeClass == 2)
        {
            InvokeCallback(m0->second, m1->second, M2(), M3());
        }

        if (m0_ok && m1_ok && m2_ok && m_sizeClass == 3)
        {
            InvokeCallback(m0->second, m1->second, m2->second, M3());
        }

        if (m0_ok && m1_ok && m2_ok && m3_ok && m_sizeClass == 4)
        {
            InvokeCallback(m0->second, m1->second, m2->second, m3->second);
        }
    }
    
private:
    template <class T>
    std::shared_ptr<T> ExactTimeValue(const double time, RingBufferBasePtr base)
    {
        std::shared_ptr<RingBuffer<T>> value =
            std::static_pointer_cast<RingBuffer<T>>(base);
        const size_t queue_size = value->QueueSize();
        for (size_t i = 0; i < queue_size; ++i)
        {
            std::shared_ptr<T> test = value->GetQueueElement(i);
            if (test != nullptr && fabs(time - test->first) < 1e-7)
            {
                return test;
            }
        }
        return nullptr;
    }
};

struct TimeHelper
{
  TimeHelper()
  {
    this->sec = 0l;
    this->nanosec = 0ul;
  }
  int32_t sec;
  uint32_t nanosec;
  // setters for named parameter idiom
  TimeHelper &set__sec(
      const int32_t &_arg)
  {
    this->sec = _arg;
    return *this;
  }
  TimeHelper &set__nanosec(
      const uint32_t &_arg)
  {
    this->nanosec = _arg;
    return *this;
  }
  double toDouble()
  {
    return this->sec + this->nanosec * 1e-9;
  }
  // comparison operators
  bool operator==(const TimeHelper &other) const
  {
    if (this->sec != other.sec)
    {
      return false;
    }
    if (this->nanosec != other.nanosec)
    {
      return false;
    }
    return true;
  }
  bool operator<(const TimeHelper &other) const
  {
    if (this->sec < other.sec)
    {
      return true;
    }
    else if (this->sec > other.sec)
    {
      return false;
    }
    if (this->nanosec < other.nanosec)
    {
      return true;
    }
    return false;
  }
  bool operator!=(const TimeHelper &other) const
  {
    return !this->operator==(other);
  }
  TimeHelper &operator-=(const TimeHelper &other)
  {
    this->sec -= other.sec;
    this->nanosec -= other.nanosec;
    return *this;
  }
  TimeHelper operator-(const TimeHelper &other)
  {
    TimeHelper rsl;
    rsl.sec -= other.sec;
    rsl.nanosec -= other.nanosec;
    return rsl;
  }
}; // struct TimeHelper


END_NS_ZF_HAL_SYNC

#endif /* ZF_HAL_EXACT_TIME_SYNC_H_ */
