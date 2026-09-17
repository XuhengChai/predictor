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
 * @brief Defines the cycle queue.
 * @return
 */

#ifndef ZF_GLOBAL_COMMON_RIGN_BUFFER_H
#define ZF_GLOBAL_COMMON_RIGN_BUFFER_H

#include <iostream>
#include <memory>
#include <vector>
#include <atomic>

#include "zf_global/zf_global.h"

BEGIN_NS_ZF

class RingBufferBase
{
};

template <typename T>
class RingBuffer : public RingBufferBase
{
public:
  explicit RingBuffer(const size_t m_vQueuesize) : m_size(m_vQueuesize)
  {
    if (m_vQueuesize == 0)
    {
    }
    m_vQueue.resize(m_size, nullptr);
    m_iIndex = 0;
  }

  RingBuffer() = delete;

  bool push(const std::shared_ptr<T> &value)
  {
    const auto current_write_pos = m_posWrite.load(std::memory_order_relaxed);
    const auto next_write_pos = (current_write_pos + 1) % m_size;
    if (next_write_pos == m_posRead.load(std::memory_order_acquire))
    {
      // LOG_ERROR() << "FULL PUSH!!!";
      return false; // 队列已满
    }
    m_vQueue[current_write_pos] = std::make_shared<Node>();
    m_vQueue[current_write_pos]->data = value;
    m_vQueue[current_write_pos]->sequence.store(next_write_pos, std::memory_order_release);
    m_posWrite.store(next_write_pos, std::memory_order_release);
    m_cntValid++; // m_cntValid.fetch_add()
    return true;
  }

  bool pop(std::shared_ptr<T> &value)
  {
    const auto current_read_pos = m_posRead.load(std::memory_order_relaxed);
    if (current_read_pos == m_posWrite.load(std::memory_order_acquire))
    {
      return false; // 队列为空
    }
    value = m_vQueue[current_read_pos]->data;
    const auto next_read_pos = (current_read_pos + 1) % m_size;
    m_posRead.store(next_read_pos, std::memory_order_release);
    m_vQueue[current_read_pos] = nullptr;
    m_cntValid--; // m_cntValid.fetch_add()
    return true;
  }

  std::shared_ptr<T> pop()
  {
    const auto current_read_pos = m_posRead.load(std::memory_order_relaxed);
    if (current_read_pos == m_posWrite.load(std::memory_order_acquire))
    {
      return nullptr; // 队列为空
    }
    std::shared_ptr<T> value;
    value = m_vQueue[current_read_pos]->data;
    const auto next_read_pos = (current_read_pos + 1) % m_size;
    m_posRead.store(next_read_pos, std::memory_order_release);
    m_vQueue[current_read_pos] = nullptr;
    m_cntValid--; // m_cntValid.fetch_add()
    return value;
  }

  int QueueSize() const { return m_size; }

  int ValidSize() const
  {
    // const auto cnt =;
    return m_cntValid.load();
  }

  int CurrentIndex() const
  {
    m_iIndex = m_posRead.load();
    return m_iIndex;
  }

  // absolute index
  const std::shared_ptr<T> GetQueueElement(const size_t index) const
  {
    if (index < 0 || index >= m_size)
    {
      return nullptr;
    }
    auto rsl = m_vQueue.at(index);
    if (rsl)
    {
      return rsl->data;
    }
    else
    {
      return nullptr;
    }
  }

  // relative index to m_posRead
  const std::shared_ptr<T> at(const size_t index) const
  {
    if (index < 0 || index >= m_size)
    {
      return nullptr;
    }
    auto tId = m_posRead.load(std::memory_order_acquire) + index;
    tId = tId % m_size; // TODO
    auto rsl = m_vQueue.at(tId);
    if (rsl)
    {
      return rsl->data;
    }
    else
    {
      return nullptr;
    }
  }

private:
  struct Node
  {
    std::shared_ptr<T> data;
    std::atomic<uint64_t> sequence;
  };

  int m_iIndex = 0;
  const size_t m_size;
  std::vector<std::shared_ptr<Node>> m_vQueue;
  // std::vector<Node> m_vQueue;
  alignas(64) std::atomic<uint64_t> m_posRead{0};
  alignas(64) std::atomic<uint64_t> m_posWrite{0};
  std::atomic<uint64_t> m_cntValid{0};
};

typedef std::shared_ptr<RingBufferBase> RingBufferBasePtr;

END_NS_ZF

#endif // !ZF_GLOBAL_COMMON_RIGN_BUFFER_H