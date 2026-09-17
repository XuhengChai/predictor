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

#ifndef ZF_TRANSPORT_SHM_RWLOCK_H_
#define ZF_TRANSPORT_SHM_RWLOCK_H_

#include <atomic>
#include <cstdint>
#include "zf_global/in/zf_framework_global.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

class RWLock
{
    friend class Segment;

public:
    RWLock();
    virtual ~RWLock() {};

    uint64_t msg_size() const { return msg_size_; }
    void set_msg_size(uint64_t msg_size) { msg_size_ = msg_size; }

    uint64_t msg_info_size() const { return msg_info_size_; }
    void set_msg_info_size(uint64_t msg_info_size)
    {
        msg_info_size_ = msg_info_size;
    }

private:
    bool TryLockForWrite();
    bool TryLockForRead();
    void ReleaseWriteLock();
    void ReleaseReadLock();
    void Init();
    void Destroy()
    {
        pthread_cond_destroy(&cond_);
        pthread_mutex_destroy(&mutex_);
        pthread_rwlock_destroy(&rwlock_);
    }
    void LockWrite() { pthread_rwlock_wrlock(&rwlock_); }
    void UnlockWrite() { pthread_rwlock_unlock(&rwlock_); }
    void LockRead() { pthread_rwlock_rdlock(&rwlock_); }
    void UnlockRead() { pthread_rwlock_unlock(&rwlock_); }

    void LockMutex() { pthread_mutex_lock(&mutex_); }
    void UnlockMutex() { pthread_mutex_unlock(&mutex_); }
    void NotifyOne() { pthread_cond_signal(&cond_); }
    void NotifyAll() { pthread_cond_broadcast(&cond_); }
    void Wait();
    void WaitFor(int32_t wait_us); // 跨进程进行条件变量等待

    pthread_rwlock_t rwlock_;
    pthread_mutex_t mutex_;
    pthread_cond_t cond_;

    uint64_t msg_size_;
    uint64_t msg_info_size_;
};

END_NS_ZF_FRAMEWORK // zf::framework

#endif // ZF_TRANSPORT_SHM_RWLOCK_H_
