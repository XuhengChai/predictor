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

#ifndef ZF_HAL_TIME_SYNC_H_
#define ZF_HAL_TIME_SYNC_H_

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <type_traits>
#include <vector>

#include "zf_global/in/hal_sync_global.h"
#include "zf_global/util/call_back.h"
#include "zf_global/util/ring_buffer.h"

// /**
//  * @namespace NS_ZF::driver::synchronization
//  */
BEGIN_NS_ZF_HAL_SYNC

struct NullType {};

// get index from TupleType
namespace detail {
template <typename T, typename Tuple, std::size_t Index = 0>
struct TupleTypeIndexHelper;

template <typename T, typename... Types, std::size_t Index>
struct TupleTypeIndexHelper<T, std::tuple<T, Types...>, Index> {
  static constexpr std::size_t value = Index;
};

template <typename T, typename U, typename... Types, std::size_t Index>
struct TupleTypeIndexHelper<T, std::tuple<U, Types...>, Index> {
  static constexpr std::size_t value =
      TupleTypeIndexHelper<T, std::tuple<Types...>, Index + 1>::value;
};
}  // namespace detail
template <typename T, typename Tuple>
constexpr std::size_t TupleTypeIndex() {
  return detail::TupleTypeIndexHelper<T, Tuple>::value;
}

// call not NullType classes
template <class... T>
struct mp_plus;
template <>
struct mp_plus<> {
  using type = std::integral_constant<int, 0>;
};
template <class T1, class... T>
struct mp_plus<T1, T...> {
  static constexpr auto _v = !T1::value + mp_plus<T...>::type::value;
  using type =
      std::integral_constant<typename std::remove_const<decltype(_v)>::type,
                             _v>;
};
template <class L, class V>
struct mp_count;
template <template <class...> class L, class... T, class V>
struct mp_count<L<T...>, V> {
  using type = typename mp_plus<std::is_same<T, V>...>::type;
};

// max 8 sensors, M0 must be the master sensor data, approximate_time
template <class M0, class M1, class M2 = NullType, class M3 = NullType,
          class M4 = NullType, class M5 = NullType, class M6 = NullType,
          class M7 = NullType>
class TimeSynchronizer : public CallBackBase {
  typedef typename mp_count<std::tuple<M0, M1, M2, M3, M4, M5, M6, M7>,
                            NullType>::type RealTypeCount;
  typedef std::tuple<M0, M1, M2, M3, M4, M5, M6, M7> Messages;

 public:
  typedef std::pair<double, M0> M0Pair;
  typedef std::pair<double, M1> M1Pair;
  typedef std::pair<double, M2> M2Pair;
  typedef std::pair<double, M3> M3Pair;
  typedef std::pair<double, M4> M4Pair;
  typedef std::pair<double, M5> M5Pair;
  typedef std::pair<double, M6> M6Pair;
  typedef std::pair<double, M7> M7Pair;
  typedef std::shared_ptr<M0Pair> M0PairPtr;
  typedef std::shared_ptr<M1Pair> M1PairPtr;
  typedef std::shared_ptr<M2Pair> M2PairPtr;
  typedef std::shared_ptr<M3Pair> M3PairPtr;
  typedef std::shared_ptr<M4Pair> M4PairPtr;
  typedef std::shared_ptr<M5Pair> M5PairPtr;
  typedef std::shared_ptr<M6Pair> M6PairPtr;
  typedef std::shared_ptr<M7Pair> M7PairPtr;
  typedef RingBuffer<M0Pair> M0RIngBuffer;
  typedef RingBuffer<M1Pair> M1RIngBuffer;
  typedef RingBuffer<M2Pair> M2RIngBuffer;
  typedef RingBuffer<M3Pair> M3RIngBuffer;
  typedef RingBuffer<M4Pair> M4RIngBuffer;
  typedef RingBuffer<M5Pair> M5RIngBuffer;
  typedef RingBuffer<M6Pair> M6RIngBuffer;
  typedef RingBuffer<M7Pair> M7RIngBuffer;
  enum EPopOrder {
    NONE_POP = 0,      // queue->pop()
    POP = 1,           // queue->pop()
    POP_DATA = 2,      // queue->pop(data)
    POP_DATA_POP = 3,  // queue->pop(data); queue->pop()
    POP_POP_DATA = 4   // queue->pop();queue->pop(data)
  };
  enum EFindType { FIND_NONE = 0, FIND_NEAR = 1, FIND_BETWEEN = 2 };
  // typedef std::shared_ptr<RingBuffer<M0Pair>> M0BufferPtr;
  TimeSynchronizer(const size_t queue_size)
      : m_fMaxScale(6.0 / 4), m_fNormalScale(3.0 / 4) {
    // m_iMasterSensor = -1;
    SetFirstSensorMaster();
    m_vDataQueue.resize(m_sizeClass, nullptr);
    m_vDataPeriod.resize(m_sizeClass, 0.036);  // 36ms for camera
    // 注意目前的last_time
    m_vDataQueue[0] = std::make_shared<RingBuffer<M0Pair>>(queue_size);
    m_vDataQueue[1] = std::make_shared<RingBuffer<M1Pair>>(queue_size);
    for (size_t i = 0; i < m_sizeClass; ++i) {
      switch (i) {
        case 2:
          m_vDataQueue[i] = std::make_shared<M2RIngBuffer>(queue_size);
          break;
        case 3:
          m_vDataQueue[i] = std::make_shared<M3RIngBuffer>(queue_size);
          break;
        case 4:
          m_vDataQueue[i] = std::make_shared<M4RIngBuffer>(queue_size);
          break;
        case 5:
          m_vDataQueue[i] = std::make_shared<M5RIngBuffer>(queue_size);
          break;
        case 6:
          m_vDataQueue[i] = std::make_shared<M6RIngBuffer>(queue_size);
          break;
        case 7:
          m_vDataQueue[i] = std::make_shared<M7RIngBuffer>(queue_size);
          break;
        default:
          break;
      }
      // m_vDataQueue[i] = std::make_shared<RingBuffer<std::pair<double,
      // std::tuple_element<i, Messages>::type>>>(queue_size);
    }
    m_vLastTimes.resize(m_sizeClass, -1);
    std::cout << "TimeSync class size: " << m_sizeClass << std::endl;
    // if (!std::is_same<M2, NullType>::value)
    // {
    //     m_vDataQueue[2] = std::make_shared<RingBuffer<M2Pair>>(queue_size);
    //     ++m_sizeClass;
    // }
  };

  void SetFirstSensorMaster() { m_iMasterSensor = 0; }
  void SetMasterSensor(int i) { m_iMasterSensor = i; }

  bool SetDataPeriod(const std::vector<double> &period) {
    if (period.size() != m_sizeClass) return false;
    m_vDataPeriod = period;
    // LOG_INFO() << m_vDataPeriod[0] << m_vDataPeriod[1];
    return true;
  }

  void SetDataPeriodI(uint32_t id, double per) {
    if (id >= m_sizeClass) return;
    m_vDataPeriod[id] = per;
  }

  template <class T>
  void PushMsgAuto(T msgData, const double time) {
    std::size_t index = TupleTypeIndex<T, Messages>();
    // std::lock_guard<std::mutex> guard(m_lock);
    std::shared_ptr<RingBuffer<std::pair<double, T>>> queue =
        std::static_pointer_cast<RingBuffer<std::pair<double, T>>>(
            m_vDataQueue[index]);
    // queue->Insert(data);
    auto data = std::make_shared<std::pair<double, T>>(time, msgData);
    queue->push(data);
    m_vLastTimes[index] = time;
    if (index == m_iMasterSensor) {
      TryToProcess(data->first);
      // TryToProcess(queue->at(0)->first);
    }
  }

  template <class T>
  void PushMsg(T msgData, const double time, int index) {
    auto data = std::make_shared<std::pair<double, T>>(time, msgData);
    // std::size_t index = TupleTypeIndex<T, Messages>();
    // LOG_DEBUG() << index << "PushMsg: " << typeid(msgData).name() << time;
    PushMsg(data, index);
  }

  void PushMsg0(const M0 &data, const double time) {
    M0PairPtr m0_ptr = std::make_shared<M0Pair>(time, data);
    PushMsg(m0_ptr, 0);
  }

  void PushMsg1(const M1 &data, const double time) {
    M1PairPtr m1_ptr = std::make_shared<M1Pair>(time, data);
    PushMsg(m1_ptr, 1);
  }

  void PushMsg2(const M2 &data, const double time) {
    M2PairPtr m2_ptr = std::make_shared<M2Pair>(time, data);
    PushMsg(m2_ptr, 2);
  }

  void PushMsg3(const M3 &data, const double time) {
    M3PairPtr m3_ptr = std::make_shared<M3Pair>(time, data);
    PushMsg(m3_ptr, 3);
  }

 protected:
  virtual void TryToProcess(const double time) {
    M0PairPtr m0 = nullptr;
    M1PairPtr m1 = nullptr;
    M2PairPtr m2 = nullptr;
    M3PairPtr m3 = nullptr;
    M4PairPtr m4 = nullptr;
    M5PairPtr m5 = nullptr;
    M6PairPtr m6 = nullptr;
    M7PairPtr m7 = nullptr;
    bool allOk[8] = {};
    bool isSuccess[8] = {true};
    EPopOrder popOrder[8] = {EPopOrder::NONE_POP};
    for (size_t i = 0; i < m_sizeClass; ++i) {
      // if (i == m_iMasterSensor)
      //{
      //	continue;
      // }
      // if (i == 0)
      //{
      //	isSuccess[0] = FindValue<M0Pair>(m0, time, i);
      //	//isSuccess &= FindValue<M0Pair>(m0, time, i);
      // }
      switch (i) {
        case 1:
          isSuccess[1] = FindValue<M1Pair>(m1, popOrder[i], time, i);
          break;
        case 2:
          isSuccess[2] = FindValue<M2Pair>(m2, popOrder[i], time, i);
          break;
        case 3:
          isSuccess[3] = FindValue<M3Pair>(m3, popOrder[i], time, i);
          break;
        case 4:
          isSuccess[4] = FindValue<M4Pair>(m4, popOrder[i], time, i);
          break;
        case 5:
          isSuccess[5] = FindValue<M5Pair>(m5, popOrder[i], time, i);
          break;
        case 6:
          isSuccess[6] = FindValue<M6Pair>(m6, popOrder[i], time, i);
          break;
        case 7:
          isSuccess[7] = FindValue<M7Pair>(m7, popOrder[i], time, i);
          break;
        default:
          break;
      }
    }
    int sum = std::accumulate(isSuccess, isSuccess + m_sizeClass, 0);
    // std::cout << "isSuccess ";
    // PrintArray(isSuccess);
    if (sum == m_sizeClass) {
      PopMsg<M0Pair>(m0, m_iMasterSensor);
      for (size_t i = 1; i < m_sizeClass; ++i) {
        switch (i) {
          case 1:
            // FindValue<M1Pair>(m1, time, i, false);
            PopValue(m1, popOrder[i], i);
            allOk[i] = (m1 != nullptr);
            break;
          case 2:
            PopValue(m2, popOrder[i], i);
            allOk[i] = (m2 != nullptr);
            break;
          case 3:
            PopValue(m3, popOrder[i], i);
            allOk[i] = (m3 != nullptr);
            break;
          case 4:
            PopValue(m4, popOrder[i], i);
            allOk[i] = (m4 != nullptr);
            break;
          case 5:
            PopValue(m5, popOrder[i], i);
            allOk[i] = (m5 != nullptr);
            break;
          case 6:
            PopValue(m6, popOrder[i], i);
            allOk[i] = (m6 != nullptr);
            break;
          case 7:
            PopValue(m7, popOrder[i], i);
            allOk[i] = (m7 != nullptr);
            break;
          default:
            break;
        }
        // m_vDataQueue[i] = std::make_shared<RingBuffer<std::pair<double,
        // std::tuple_element<i, Messages>::type>>>(queue_size);
      }
    } else {
      return;
    }
    allOk[0] = (m0 != nullptr);
    sum = std::accumulate(allOk, allOk + m_sizeClass, 0);
    // std::cout << "all ok ";
    // PrintArray(allOk);
    // LOG_DEBUG() << "sum" << sum << "isSuccess" << isSuccess << "\n";

    switch (m_sizeClass) {
      case 2:
        if (sum == m_sizeClass) {
          InvokeCallback(m0->second, m1->second);
        }
        break;
      case 3:
        if (sum == m_sizeClass) {
          InvokeCallback(m0->second, m1->second, m2->second);
        }
        break;
      case 4:
        if (sum == m_sizeClass) {
          InvokeCallback(m0->second, m1->second, m2->second, m3->second);
        }
        break;
      case 5:
        if (sum == m_sizeClass) {
          InvokeCallback(m0->second, m1->second, m2->second, m3->second,
                         m4->second);
        }
        break;
      case 6:
        if (sum == m_sizeClass) {
          InvokeCallback(m0->second, m1->second, m2->second, m3->second,
                         m4->second, m5->second);
        }
        break;
      case 7:
        if (sum == m_sizeClass) {
          InvokeCallback(m0->second, m1->second, m2->second, m3->second,
                         m4->second, m5->second, m6->second);
        }
        break;
      case 8:
        if (sum == m_sizeClass) {
          InvokeCallback(m0->second, m1->second, m2->second, m3->second,
                         m4->second, m5->second, m6->second, m7->second);
        }
        break;
      default:
        break;
    }
  }

  template <class T>
  bool FindValue(std::shared_ptr<T> &data, EPopOrder &popOrder,
                 const double sync_time, const uint32_t &index) {
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    if (index == m_iMasterSensor) {
      return true;
    }
    // std::lock_guard<std::mutex> guard(m_lock);
    std::shared_ptr<RingBuffer<T>> UnsyncedData =
        std::static_pointer_cast<RingBuffer<T>>(m_vDataQueue[index]);
    // LOG_INFO() << index << ": " << std::to_string(sync_time).c_str() << "
    // UnsyncedData size1: " << UnsyncedData->ValidSize();
    EFindType findType = EFindType::FIND_NONE;
    while (UnsyncedData->ValidSize() >= 2) {
      // LOG_INFO() << index << sync_time << " UnsyncedData size2: " <<
      // UnsyncedData->ValidSize();
      double time0 = UnsyncedData->at(0)->first;
      double time1 = UnsyncedData->at(1)->first;
      // LOG_INFO() << "time0: " << std::to_string(time0).c_str()
      // << "time1: " << std::to_string(time1).c_str()
      // << " AND: " << std::to_string(sync_time).c_str();

      // find nearst data -- canbe comment when you want to use interpolation
      if (fabs(time0 - sync_time) < (m_vDataPeriod[index] / 3)) {
        // UnsyncedData->pop(data);
        findType = EFindType::FIND_NEAR;
        popOrder = EPopOrder::POP_DATA;
        break;
      }

      if (fabs(time1 - sync_time) < (m_vDataPeriod[index] / 3)) {
        // UnsyncedData->pop();     // pop 0
        // UnsyncedData->pop(data); // pop 1
        findType = EFindType::FIND_NEAR;
        popOrder = EPopOrder::POP_POP_DATA;
        break;
      }

      // later than time, return, drop radar
      if (time0 > sync_time) {
        if ((time0 - sync_time) <= (m_fNormalScale * m_vDataPeriod[index])) {
          // UnsyncedData->pop(data); // pop 0
          findType = EFindType::FIND_NEAR;
          popOrder = EPopOrder::POP_DATA;
          break;
        }
        // LOG_INFO() << index << ", time0: " << time0 << "time1: " << time1 <<
        // " AND: " << sync_time;
        PopMasterMsg();  // drop master radar data
        LOG_ERROR() << index << ", time0: " << std::to_string(time0)
                    << "time1: " << std::to_string(time1)
                    << " AND: " << std::to_string(sync_time)
                    << "drop master radar data 1##############";
        return false;
      }
      // else at(0) early than time

      // at(1) early than time, continue, find another
      if (time1 < sync_time) {
        UnsyncedData->pop();
        continue;
      }
      // else: between UnsyncedData..at(0) and UnsyncedData.at(1)

      // missing datas, time0 exceed max time error limit
      if (sync_time - time0 > (m_fMaxScale * m_vDataPeriod[index]))  // 0.2?
      {
        // time1 in normal time error limit
        if (time1 - sync_time <= (m_fNormalScale * m_vDataPeriod[index])) {
          // UnsyncedData->pop();     // pop 0
          // UnsyncedData->pop(data); // pop 1
          findType = EFindType::FIND_NEAR;
          popOrder = EPopOrder::POP_POP_DATA;
          break;
        }
        // time1 exceed normal time error limit
        else {
          PopMasterMsg();  // drop master radar data
          LOG_WARN() << index << ", time0: " << std::to_string(time0)
                     << "time1: " << std::to_string(time1)
                     << " AND: " << std::to_string(sync_time)
                     << " drop master radar data 2";
          return false;
        }
      }

      // missing datas, time1 exceed max time error limit
      if (time1 - sync_time > (m_fMaxScale * m_vDataPeriod[index])) {
        // time0 in normal time error limit
        if (sync_time - time0 <= (m_fNormalScale * m_vDataPeriod[index])) {
          // UnsyncedData->pop(data); // pop 0
          // UnsyncedData->pop();     // pop 1
          findType = EFindType::FIND_NEAR;
          popOrder = EPopOrder::POP_DATA_POP;
          break;
        }
        // time0 exceed normal time error limit
        else {
          PopMasterMsg();  // drop master radar data
          LOG_WARN() << "drop master radar data 3";
          return false;
        }
      }
      findType = EFindType::FIND_BETWEEN;
      break;
    }
    if (UnsyncedData->ValidSize() < 2) return false;
    if (findType == EFindType::FIND_NEAR) {
      return true;
    }

    auto front_data = UnsyncedData->at(0);
    auto back_data = UnsyncedData->at(1);

    // TODO: interpolation

    if (fabs(front_data->first - sync_time) <=
        fabs(back_data->first - sync_time)) {
      // UnsyncedData->pop(data); // pop 0
      popOrder = EPopOrder::POP_DATA;
    } else {
      // UnsyncedData->pop();     // pop 0
      // UnsyncedData->pop(data); // pop 1
      popOrder = EPopOrder::POP_POP_DATA;
    }
    return true;
  }

  template <class T>
  bool PopValue(std::shared_ptr<T> &data, const EPopOrder &popOrder,
                uint32_t index) {
    if (index >= m_sizeClass) {
      return false;
    }
    std::shared_ptr<RingBuffer<T>> UnsyncedData =
        std::static_pointer_cast<RingBuffer<T>>(m_vDataQueue[index]);
    // pop delay one data;
    switch (popOrder) {
      case POP_DATA:
        // UnsyncedData->pop(data);// pop 0
        data = UnsyncedData->at(0);
        break;
      case POP_POP_DATA:
        UnsyncedData->pop();  // pop 0
        // UnsyncedData->pop(data); // pop 1
        data = UnsyncedData->at(0);
        break;
      case POP_DATA_POP:
        UnsyncedData->pop(data);  // pop 0
        // UnsyncedData->pop();	 // pop 1
        break;
      default:
        break;
    }
    return true;
  }

  template <class T>
  void PushMsg(std::shared_ptr<std::pair<double, T>> data, const int index) {
    std::shared_ptr<RingBuffer<std::pair<double, T>>> queue =
        std::static_pointer_cast<RingBuffer<std::pair<double, T>>>(
            m_vDataQueue[index]);
    // LOG_DEBUG() << "shared_ptr: " << m_vDataQueue[index].get();
    // m_vDataQueue[index] = queue;
    {
      std::lock_guard<std::mutex> guard(m_lock);
      if (!queue->push(data)) {
        LOG_ERROR() << index << ", " << std::to_string(data->first)
                    << ", FULL PUSH!";
      }
    }
    if (index == m_iMasterSensor)  // TODO
    {
      m_vLastTimes[index] = queue->at(0)->first;
      m_iCntMasterMsg++;
      // const auto cnt1 = m_iCntMasterMsg.load(std::memory_order_relaxed);
      // LOG_DEBUG()  << cnt1 << " m_vLastTimes: " << m_vLastTimes[index] << ",
      // " << data->first;
    }
    const auto cnt = m_iCntMasterMsg.load(std::memory_order_relaxed);
    if (cnt) {
      std::lock_guard<std::mutex> guard(m_lock);
      // LOG_DEBUG() << cnt << " m_vLastTimes: " <<
      // m_vLastTimes[m_iMasterSensor] << ", " << data->first;
      TryToProcess(m_vLastTimes[m_iMasterSensor]);
    }
  }

  double MinTime() {
    return *std::min_element(m_vLastTimes.begin(), m_vLastTimes.end());
  }

  double MasterTime() {
    if (m_iMasterSensor == -1) {
      return *std::min_element(m_vLastTimes.begin(), m_vLastTimes.end());
    } else {
      return m_vLastTimes.at(m_iMasterSensor);
    }
  }

  void PopMasterMsg() {
    // PopMsg<M0Pair>(0);
    PopMsg<M0Pair>(m_iMasterSensor);
  }

  template <class T>
  bool PopMsg(std::shared_ptr<T> &data, uint32_t id) {
    if (id >= m_sizeClass) {
      return false;
    }
    std::shared_ptr<RingBuffer<T>> queue =
        std::static_pointer_cast<RingBuffer<T>>(m_vDataQueue[id]);
    // LOG_DEBUG() << "queue->name " << typeid(data).name();
    // LOG_DEBUG() << id << " queue->ValidSize " << queue->ValidSize();
    bool isPoped = queue->pop(data);
    if (id == m_iMasterSensor && isPoped)  // TODO
    {
      m_iCntMasterMsg--;
      if (queue->ValidSize())  // TODO
      {
        m_vLastTimes[m_iMasterSensor] = queue->at(0)->first;
      }
    }
    return isPoped;
  }

  template <class T>
  std::shared_ptr<T> PopMsg(uint32_t id) {
    std::shared_ptr<T> data = nullptr;
    this->PopMsg<T>(data, id);
    return data;
  }

 protected:
  int m_iMasterSensor;
  std::atomic<uint32_t> m_iCntMasterMsg{0};

  std::mutex m_lock;
  std::vector<RingBufferBasePtr> m_vDataQueue;
  std::vector<double> m_vLastTimes;
  std::vector<double> m_vDataPeriod;  // unit: ms
  double m_dLastTime;
  RealTypeCount m_sizeClass;
  float m_fMaxScale;
  float m_fNormalScale;
};

END_NS_ZF_HAL_SYNC

#endif /* ZF_HAL_TIME_SYNC_H_ */
