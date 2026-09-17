/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "zf_framework_transport/shm/xsi_segment.h"

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>

#include "zf_framework_transport/shm/segment.h"
#include "zf_framework_transport/shm/shm_conf.h"

#include "zf_global/in/zf_framework_global.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

XsiSegment::XsiSegment(uint64_t channel_id) : Segment(channel_id)
{
  key_ = static_cast<key_t>(channel_id);
  shm_size_ =  conf_.managed_shm_size();
  // LOG_DEBUG() << "XsiSegment key: " << key_;
}

XsiSegment::~XsiSegment() { Destroy(); }


bool XsiSegment::OpenOrCreate()
{
  // Remove();
  // return true;
  if (init_)
  {
    return true;
  }

  // create managed_shm_
  int retry = 0;
  int shmid = 0;
  while (retry < 2)
  {
    shmid = shmget(key_, shm_size_, 0644 | IPC_CREAT | IPC_EXCL);
    if (shmid != -1)
    {
      break;
    }

    if (EINVAL == errno)
    {
      LOG_INFO() << "need larger space, recreate.";
      Reset();
      Remove();
      ++retry;
    }
    else if (EEXIST == errno)
    {
      LOG_DEBUG() << "shm already exist, open only.";
      return OpenOnly();
    }
    else
    {
      break;
    }
  }

  if (shmid == -1)
  {
    LOG_ERROR() << "create shm failed, error code: " << strerror(errno);
    return false;
  }

  // attach managed_shm_
  managed_shm_ = shmat(shmid, nullptr, 0);
  if (managed_shm_ == reinterpret_cast<void *>(-1))
  {
    LOG_ERROR() << "attach shm failed, error: " << strerror(errno);
    shmctl(shmid, IPC_RMID, 0);
    return false;
  }
  // LOG_DEBUG() << "managed_shm_ is: " << managed_shm_  << "  shm_size_: "<< shm_size_;

  if (!CreateShm(shmid))
  {
    return false;
  }

  init_ = true;
  // LOG_DEBUG() << "open or create true.";
  return true;
}

bool XsiSegment::OpenOnly()
{
  if (init_)
  {
    return true;
  }

  // get managed_shm_
  int shmid = shmget(key_, 0, 0644);
  if (shmid == -1)
  {
    LOG_ERROR() << "get shm failed. error: " << strerror(errno);
    return false;
  }

  // attach managed_shm_
  managed_shm_ = shmat(shmid, nullptr, 0);
  if (managed_shm_ == reinterpret_cast<void *>(-1))
  {
    LOG_ERROR() << "attach shm failed, error: " << strerror(errno);
    return false;
  }
  // LOG_DEBUG() << "managed_shm_ is: " << managed_shm_;

  if (!OpenShm(shmid))
  {
    return false;
  }

  init_ = true;
  LOG_DEBUG() << "open only true.";
  return true;
}

bool XsiSegment::CreateShm(int shmid)
{
  // create field state_
  state_ = new (managed_shm_) State(conf_.ceiling_msg_size());
  if (state_ == nullptr)
  {
    LOG_ERROR() << "create state failed.";
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
    shmctl(shmid, IPC_RMID, 0);
    return false;
  }

  conf_.Update(state_->ceiling_msg_size());

  // create field blocks_
  blocks_ = new (static_cast<char *>(managed_shm_) + sizeof(State))
      Block[conf_.block_num()];
  if (blocks_ == nullptr)
  {
    LOG_ERROR() << "create blocks failed.";
    state_->~State();
    state_ = nullptr;
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
    shmctl(shmid, IPC_RMID, 0);
    return false;
  }

  // create block buf
  uint32_t i = 0;
  for (; i < conf_.block_num(); ++i)
  {
    uint8_t *addr =
        new (static_cast<char *>(managed_shm_) + sizeof(State) +
             conf_.block_num() * sizeof(Block) + i * conf_.block_buf_size())
            uint8_t[conf_.block_buf_size()];

    std::lock_guard<std::mutex> _g(block_buf_lock_);
    block_buf_addrs_[i] = addr;
  }

  if (i != conf_.block_num())
  {
    LOG_ERROR() << "create block buf failed.";
    state_->~State();
    state_ = nullptr;
    blocks_ = nullptr;
    {
      std::lock_guard<std::mutex> _g(block_buf_lock_);
      block_buf_addrs_.clear();
    }
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
    shmctl(shmid, IPC_RMID, 0);
    return false;
  }
  state_->IncreaseReferenceCounts();
  return true;
}

bool XsiSegment::OpenShm(int shmid)
{

  // get field state_
  state_ = reinterpret_cast<State *>(managed_shm_);
  if (state_ == nullptr)
  {
    LOG_ERROR() << "get state failed.";
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
    return false;
  }

  conf_.Update(state_->ceiling_msg_size());

  // get field blocks_
  blocks_ = reinterpret_cast<Block *>(static_cast<char *>(managed_shm_) +
                                      sizeof(State));
  if (blocks_ == nullptr)
  {
    LOG_ERROR() << "get blocks failed.";
    state_ = nullptr;
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
    return false;
  }

  // get block buf
  uint32_t i = 0;
  for (; i < conf_.block_num(); ++i)
  {
    uint8_t *addr = reinterpret_cast<uint8_t *>(
        static_cast<char *>(managed_shm_) + sizeof(State) +
        conf_.block_num() * sizeof(Block) + i * conf_.block_buf_size());

    if (addr == nullptr)
    {
      break;
    }
    std::lock_guard<std::mutex> _g(block_buf_lock_);
    block_buf_addrs_[i] = addr;
  }

  if (i != conf_.block_num())
  {
    LOG_ERROR() << "open only failed.";
    state_->~State();
    state_ = nullptr;
    blocks_ = nullptr;
    {
      std::lock_guard<std::mutex> _g(block_buf_lock_);
      block_buf_addrs_.clear();
    }
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
    shmctl(shmid, IPC_RMID, 0);
    return false;
  }
  state_->IncreaseReferenceCounts();
  return true;
}

bool XsiSegment::Remove()
{
  int shmid = shmget(key_, 0, 0644);
  if (shmid == -1 || shmctl(shmid, IPC_RMID, 0) == -1)
  {
    LOG_ERROR() << "remove shm failed, error code: " << strerror(errno);
    return false;
  }
  LOG_DEBUG() << "remove success.";
  return true;
}

void XsiSegment::Reset()
{
  state_ = nullptr;
  blocks_ = nullptr;
  {
    std::lock_guard<std::mutex> _g(block_buf_lock_);
    block_buf_addrs_.clear();
  }
  if (managed_shm_ != nullptr)
  {
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
    return;
  }
}

END_NS_ZF_FRAMEWORK // zf::framework