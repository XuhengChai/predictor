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

#include "zf_framework_transport/shm/rwlock.h"
#include <sys/time.h>//gettimeofday

BEGIN_NS_ZF_FRAMEWORK // zf::framework

RWLock::RWLock()
{
  Init();
}

bool RWLock::TryLockForWrite()
{
  if (pthread_rwlock_trywrlock(&rwlock_) == 0)
  {
    return true;
  }
  return false;
}

bool RWLock::TryLockForRead()
{
  if (pthread_rwlock_tryrdlock(&rwlock_) == 0)
  {
    return true;
  }
  return false;
}

void RWLock::ReleaseWriteLock()
{
  UnlockWrite();
}

void RWLock::ReleaseReadLock()
{
  UnlockRead();
}

void RWLock::Init()
{
  pthread_rwlockattr_t rwattr;
  pthread_rwlockattr_init(&rwattr);
  pthread_rwlockattr_setpshared(&rwattr, PTHREAD_PROCESS_SHARED);
  pthread_rwlockattr_setkind_np(&rwattr, PTHREAD_RWLOCK_PREFER_WRITER_NONRECURSIVE_NP);
  pthread_rwlock_init(&rwlock_, &rwattr);

  pthread_mutexattr_t mutexattr; // 设置 mutex 的 PTHREAD_PROCESS_SHARED 属性
  pthread_mutexattr_init(&mutexattr);
  pthread_mutexattr_setpshared(&mutexattr, PTHREAD_PROCESS_SHARED);
  pthread_mutex_init(&mutex_, &mutexattr);

  pthread_condattr_t condattr; // 设置 cond 的 PTHREAD_PROCESS_SHARED 属性
  pthread_condattr_init(&condattr);
  pthread_condattr_setpshared(&condattr, PTHREAD_PROCESS_SHARED);
  pthread_cond_init(&cond_, &condattr);
}

void RWLock::Wait()
{
  LockMutex();
  pthread_cond_wait(&cond_, &mutex_);
  UnlockMutex();
}

void RWLock::WaitFor(int32_t wait_us)
{
  if (!wait_us)
  {
    Wait();
    return;
  }
  // LOG_DEBUG() << "_data is" << _data;
  struct timeval now;
  struct timespec abstime;
  gettimeofday(&now, NULL);
  // printf("wait sec:%ld,nsec:%ld,wait_ms:%d\r\n", now.tv_sec, now.tv_usec * 1000, wait_ms);
  abstime.tv_nsec = (now.tv_usec + wait_us) * 1000;
  abstime.tv_sec = now.tv_sec + (abstime.tv_nsec) / 1000000000;
  abstime.tv_nsec = abstime.tv_nsec % 1000000000;
  pthread_mutex_lock(&mutex_);
  // printf("wait sec:%ld,nsec:%ld\r\n", abstime.tv_sec, abstime.tv_nsec);
  // pthread_cond_wait(&_shm_mutex->_cond,&_shm_mutex->_mutex);
  pthread_cond_timedwait(&cond_, &mutex_, &abstime);

  pthread_mutex_unlock(&mutex_);
}

END_NS_ZF_FRAMEWORK // zf::framework
