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

#include "zf_framework_transport/shm/condition_notifier.h"

#include <sys/ipc.h>
#include <sys/shm.h>
#include <thread>

BEGIN_NS_ZF_FRAMEWORK

ConditionNotifier::ConditionNotifier() : ConditionNotifierSegBase(-1)
{
  key_ = static_cast<key_t>(std::hash<std::string>{}("/zf/framework/transport/shm/notifier"));
  // key_ = static_cast<key_t>(std::hash<std::string>{}("send_receiver_img"));
  // LOG_DEBUG() << "condition notifier key: " << key_;
  shm_name_ = std::to_string(key_);
  // LOG_DEBUG() << "condition notifier shm_name_: " << shm_name_.c_str();
  shm_size_ = sizeof(Indicator);
  // LOG_DEBUG() << "shm_size_: " << shm_size_;

  if (!Init())
  {
    LOG_ERROR() << "fail to init condition notifier.";
    is_shutdown_.store(true);
    return;
  }
  next_seq_ = indicator_->next_seq.load();
  // LOG_DEBUG() << "next_seq: " << next_seq_;
}

ConditionNotifier::~ConditionNotifier() { Shutdown(); }

void ConditionNotifier::Shutdown()
{
  LOG_DEBUG() << "ConditionNotifier. Shutdown begin";
  if (is_shutdown_.exchange(true))
  {
    return;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  Reset();
  LOG_DEBUG() << "ConditionNotifier. Shutdown end";
}

bool ConditionNotifier::Notify(const ReadableInfo &info)
{
  if (is_shutdown_.load())
  {
    LOG_DEBUG() << "notifier is shutdown.";
    return false;
  }

  uint64_t seq = indicator_->next_seq.fetch_add(1);
  uint64_t idx = seq % kBufLength;
  indicator_->infos[idx] = info;
  indicator_->seqs[idx] = seq;
  // LOG_DEBUG() << idx << " notifier ConditionNotifier." << indicator_->next_seq.load();
  return true;
}

bool ConditionNotifier::Listen(int timeout_ms, ReadableInfo *info)
{
  if (info == nullptr)
  {
    LOG_ERROR() << "info nullptr.";
    return false;
  }

  if (is_shutdown_.load())
  {
    LOG_DEBUG() << "notifier is shutdown.";
    return false;
  }

  int timeout_us = timeout_ms * 1000;
  while (!is_shutdown_.load())
  {
    uint64_t seq = indicator_->next_seq.load();
    if (seq != next_seq_)
    {
      auto idx = next_seq_ % kBufLength;
      auto actual_seq = indicator_->seqs[idx];

      if (actual_seq >= next_seq_)
      {
        next_seq_ = actual_seq;
        *info = indicator_->infos[idx];
        ++next_seq_;
        return true;
      }
      else
      {
        LOG_DEBUG() << "seq[" << next_seq_ << "] is writing, can not read now.";
      }
    }

    if (timeout_us > 0)
    {
      std::this_thread::sleep_for(std::chrono::microseconds(50));
      timeout_us -= 50;
    }
    else
    {
      return false;
    }
  }
  return false;
}

bool ConditionNotifier::CreateShm(int shmid)
{
  // create indicator_
  // LOG_DEBUG() << "create indicator." << managed_shm_ << " ADN " << sizeof(Indicator);
  indicator_ = new (managed_shm_) Indicator();
  if (indicator_ == nullptr)
  {
    LOG_ERROR() << "create indicator failed.";
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
    shmctl(shmid, IPC_RMID, 0);
    return false;
  }
  return true;
}

bool ConditionNotifier::OpenShm(int shmid)
{
  // get indicator_
  indicator_ = reinterpret_cast<Indicator *>(managed_shm_);
  if (indicator_ == nullptr)
  {
    LOG_ERROR() << "get indicator failed.";
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
    return false;
  }
  LOG_DEBUG() << "open true.";
  return true;
}

bool ConditionNotifier::Init() { return OpenOrCreate(); }

void ConditionNotifier::Reset()
{
  indicator_ = nullptr;
  if (managed_shm_ != nullptr)
  {
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
  }
}

END_NS_ZF_FRAMEWORK
