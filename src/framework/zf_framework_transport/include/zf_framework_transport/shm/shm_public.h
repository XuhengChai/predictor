#ifndef _SHM_PUBLIC_H
#define _SHM_PUBLIC_H 1

#include <string.h>
#include <time.h>
#include <signal.h>
#include <errno.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <sys/mman.h> // for shm_open
#include <sys/stat.h> /* For mode constants */
#include <fcntl.h>    /* For O_* constants */
#include <pthread.h>  // for pthread_xx
#include <assert.h>   // for assert

#include <atomic> // for atomic
#include <cstdint>
#include <mutex>
#include <string>
#include <functional>

#include "zf_global/in/zf_framework_global.h"

// #define PROJECT_ID 'K'
// #define FLAG_SERVER 0 // indicates the current operation is from server
// #define FLAG_CLIENT 1 // indicates the current operation is from client

BEGIN_NS_ZF_FRAMEWORK // zf::framework

    const uint32_t BLOCK_NUM_16K = 512;
const uint64_t MESSAGE_SIZE_16K = 1024 * 16;

class State
{
public:
    explicit State(const uint64_t &ceiling_msg_size) : ceiling_msg_size_(ceiling_msg_size){};
    virtual ~State() = default;

    void DecreaseReferenceCounts()
    {
        uint32_t current_reference_count = reference_count_.load();
        do
        {
            if (current_reference_count == 0)
            {
                return;
            }
        } while (!reference_count_.compare_exchange_strong(
            current_reference_count, current_reference_count - 1));
    }

    void IncreaseReferenceCounts() { reference_count_.fetch_add(1); }

    uint32_t FetchAddSeq(uint32_t diff) { return seq_.fetch_add(diff); }
    uint32_t seq() { return seq_.load(); }

    void set_need_remap(bool need) { need_remap_.store(need); }
    bool need_remap() { return need_remap_; }

    uint64_t ceiling_msg_size() { return ceiling_msg_size_.load(); }
    uint32_t reference_counts() { return reference_count_.load(); }

private:
    std::atomic<bool> need_remap_ = {false};
    std::atomic<uint32_t> seq_ = {0};
    std::atomic<uint32_t> reference_count_ = {0};
    std::atomic<uint64_t> ceiling_msg_size_;
}; // sizeof(State) 32

struct ShmBlockInfo
{
    bool written_;
    long timestamp_ = {};
    size_t size_;
    char data_[1];

    ShmBlockInfo() : written_(false) {}

    void Write(const char *data, const size_t len)
    {
        written_ = false;
        memcpy(data_, data, len);
        size_ = len;
        timestamp_ = GetTimestamp();
        written_ = true; // has been written
    }

    bool Read(std::vector<char> *data, long *time = nullptr)
    {
        if (!written_) // has been written
        {
            return false;
        }
        if (time)
        {
            *time = timestamp_;
        }
        data->resize(size_);
        memcpy(data->data(), data_, size_);
        return true;
    }

    static long GetTimestamp()
    {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        return ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
    }
};

struct ShmBlockQueue // 跨进程通信时使用的结构体
{
    size_t block_size_;
    int block_cnt_;
    int msg_size_;
    int cur_index_;
    char data_[1];

    // a block include a header: ShmBlockInfo, and the real msg data
    // the size of a block should be fixed now
    // TODO: judge the len
    ShmBlockQueue(const size_t msg_size, const int block_cnt)
        : block_size_(sizeof(ShmBlockInfo) + msg_size),
          msg_size_(msg_size),
          block_cnt_(block_cnt), cur_index_(0)
    {
        new (data_) ShmBlockInfo();
    }

    void Write(const char *data, const size_t len)
    {
        const int next = (cur_index_ + 1) % block_cnt_;
        if (len <= msg_size_)
        {
            (reinterpret_cast<ShmBlockInfo *>(data_ + next * block_size_))->Write(data, len);
        }
        else
        {
            LOG_ERROR() << "SHM: exceed max msg Length";
            (reinterpret_cast<ShmBlockInfo *>(data_ + next * block_size_))->Write(data, msg_size_);
        }
        cur_index_ = next;
    }

    bool Read(std::vector<char> *data, long *time)
    {
        return (reinterpret_cast<ShmBlockInfo *>(data_ + cur_index_ * block_size_))->Read(data, time);
    }
};

struct ShmSlice
{
    int attached_;
    pthread_rwlock_t rwlock_;
    pthread_mutex_t mutex_;
    pthread_cond_t cond_;
    char data_[1];

    ShmSlice(const size_t msg_size, const int block_cnt, const bool init = false)
    {
        if (init)
        {
            // // init rwlock
            // pthread_rwlockattr_t rwattr;
            // pthread_rwlockattr_init(&rwattr);
            // pthread_rwlockattr_setpshared(&rwattr, PTHREAD_PROCESS_SHARED);
            // pthread_rwlock_init(&rwlock_, &rwattr);
            // // init mutex
            // pthread_mutexattr_t mattr;
            // pthread_mutexattr_init(&mattr);
            // pthread_mutexattr_setpshared(&mattr, PTHREAD_PROCESS_SHARED);
            // pthread_mutex_init(&mutex_, &mattr);

            // // init condition variable
            // pthread_condattr_t cattr;
            // pthread_condattr_init(&cattr);
            // pthread_condattr_setpshared(&cattr, PTHREAD_PROCESS_SHARED);
            // pthread_cond_init(&cond_, &cattr);

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

            // init shm queue
            new (data_) ShmBlockQueue(msg_size, block_cnt);
            // 如果你想在已经分配的内存中创建一个对象，使用new时行不通的。
            // 也就是说placement new允许你在一个已经分配好的内存中（栈或者堆中）构造一个新的对象。原型中void* p实际上就是指向一个已经分配好的内存缓冲区的的首地址。
        }
    }
    ~ShmSlice()
    {
    }

    void Destroy()
    {
        pthread_cond_destroy(&cond_);
        pthread_mutex_destroy(&mutex_);
        pthread_rwlock_destroy(&rwlock_);
    }

    void Write(const char *data, const size_t len)
    {
        // LOG_DEBUG() << "LockWrite";
        LockWrite();
        (reinterpret_cast<ShmBlockQueue *>(data_))->Write(data, len);
        UnlockWrite();
        // LOG_DEBUG() << "UnlockWrite";
    }
    bool Read(std::vector<char> *data, long *time)
    {
        return (reinterpret_cast<ShmBlockQueue *>(data_))->Read(data, time);
    }

    void LockWrite() { pthread_rwlock_wrlock(&rwlock_); }
    void UnlockWrite() { pthread_rwlock_unlock(&rwlock_); }
    void LockRead() { pthread_rwlock_rdlock(&rwlock_); }
    void UnlockRead() { pthread_rwlock_unlock(&rwlock_); }

    void LockMutex()
    {
        pthread_mutex_lock(&mutex_);
        // while (EOWNERDEAD == pthread_mutex_lock(&mutex_))
        // {
        //     UnlockMutex();
        // }
    }
    void UnlockMutex() { pthread_mutex_unlock(&mutex_); }
    void NotifyOne() { pthread_cond_signal(&cond_); }
    void NotifyAll() { pthread_cond_broadcast(&cond_); }
    void Wait()
    {
        LockMutex();
        pthread_cond_wait(&cond_, &mutex_);
        UnlockMutex();
    }

    void WaitFor(int32_t wait_us) // 跨进程进行条件变量等待
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

    void Notify(const std::vector<char> &data)
    {
        // LockMutex();
        Write(data.data(), data.size());
        // UnlockMutex();
        NotifyAll();
        // pthread_mutex_lock(&this->mutex_);
        // printf("notify..\r\n");
        // pthread_cond_broadcast(&this->cond_);
        // pthread_mutex_unlock(&this->mutex_);
        // UnlockMutex();
        // LOG_DEBUG() << "NotifyAll";
    }

    void Notify(const char *data, const size_t len)
    {
        // LockMutex();
        Write(data, len);
        // UnlockMutex();
        NotifyAll();
        // pthread_mutex_lock(&this->mutex_);
        // printf("notify..\r\n");
        // pthread_cond_broadcast(&this->cond_);
        // pthread_mutex_unlock(&this->mutex_);
        // UnlockMutex();
        // LOG_DEBUG() << "NotifyAll";
    }

    bool Listen(int timeout_ms, std::vector<char> &data, long &last_read_time)
    {
        // LOG_DEBUG() << "1. last_read_time is" << last_read_time;

        std::vector<char> data_read;
        long time(0);
        Read(&data_read, &time);
        // LOG_DEBUG() << "1. time is" << time << ", last_read_time is" << last_read_time;
        // LOG_DEBUG() << "1. time is" << time << ", last_read_time is" << last_read_time;
        if (time > last_read_time)
        {
            last_read_time = time;
            // LOG_DEBUG() << "Listen true 1; ";
            data.swap(data_read);
            return true;
        }
        int timeout_us = timeout_ms * 1000;
        WaitFor(timeout_us);
        Read(&data_read, &time);
        // LOG_DEBUG() << "2. WaitFor time is" << time << ", last_read_time is" << last_read_time;
        if (time > last_read_time)
        {
            last_read_time = time;
            // LOG_DEBUG() << "Listen true 2; ";
            data.swap(data_read);
            return true;
        }
        return false;
    }

    /*
    long read_time = 0;
    while (running_)
    {
      std::vector<char> data;
      long time;
      struct timespec ts;
      clock_gettime(CLOCK_REALTIME, &ts);
      ts.tv_sec += 5;
      if (!slice_->WaitFor(&ts, [&]
                           { return slice_->Read(&data, &time) && time > read_time; }))
      {
        continue;
      }
      read_time = time;
      callback_(data);
    */
    bool Listen(int timeout_ms, const std::function<bool()> &cond = nullptr)
    {
        if (cond && cond())
        {
            return true;
        }
        int timeout_us = timeout_ms * 1000;
        struct timeval now;
        struct timespec abstime;
        gettimeofday(&now, NULL);
        abstime.tv_nsec = (now.tv_usec + timeout_us) * 1000;
        abstime.tv_sec = now.tv_sec + (abstime.tv_nsec) / 1000000000;
        abstime.tv_nsec = abstime.tv_nsec % 1000000000;

        LockMutex();
        pthread_cond_timedwait(&cond_, &mutex_, &abstime);
        UnlockMutex();
        bool ret;
        if (cond)
        {
            ret = cond();
        }
        else
        {
            struct timespec now;
            clock_gettime(CLOCK_REALTIME, &now);
            ret = now.tv_sec < abstime.tv_sec ||
                  (now.tv_sec == abstime.tv_sec && now.tv_nsec <= abstime.tv_nsec);
        }
        return ret;
    }
};

class FRAMEWORK_TRANS_EXPORT PosixSegmentMutex
{
private:
    std::string shm_name_;
    bool init_ = {};
    void *managed_shm_{nullptr};
    State *state_;
    ShmSlice *slice_{nullptr};
    // uint8_t *_data{nullptr};
    size_t shm_size;
    size_t m_msgSize;
    size_t m_blockCount;
    long m_lastReadTime = {};
    int m_timeoutMs;

public:
    PosixSegmentMutex(std::string path);

    ~PosixSegmentMutex();

    bool Notify(const std::vector<char> &data)
    {
        if (!init_)
        {
            LOG_ERROR() << "create shm failed, can't write now.";
            return false;
        }
        // LOG_DEBUG() << "Notify ." << slice_;
        // LOG_DEBUG();
        slice_->Notify(data);
        return true;
    }

    bool Notify(const char *data, const size_t len)
    {
        if (!init_)
        {
            LOG_ERROR() << "create shm failed, can't write now.";
            return false;
        }
        slice_->Notify(data, len);
        return true;
    }

    bool Listen(std::vector<char> &data)
    {
        if (!init_)
        {
            // LOG_ERROR() << "create shm failed, can't read now.";
            return false;
        }
        // printf("\n%d, Listen msg ... \r\n", getpid());
        bool ret = slice_->Listen(m_timeoutMs, data, m_lastReadTime);
        // LOG_INFO() << "ret******" << ret << " ---" << data.size() << "---";

        return ret;
    }

    void SetTimeout(uint32_t timeout_ms = 5000) // 1 second
    {
        m_timeoutMs = timeout_ms;
    }

    bool OpenOrCreate(size_t msg_size = 512, size_t block_count = 512)
    {
        if (init_)
        {
            return true;
        }
        // slice_size = sizeof(ShmSlice) + sizeof(ShmBlockQueue) + (sizeof(ShmBlockInfo) + msg_size) * block_count;
        m_msgSize = msg_size;
        m_blockCount = block_count;
        shm_size = sizeof(State) + sizeof(ShmSlice) + sizeof(ShmBlockQueue) + (sizeof(ShmBlockInfo) + msg_size) * block_count; // 计算内存大小

        int shm_fd = shm_open(shm_name_.c_str(), O_RDWR | O_CREAT | O_EXCL, 0644); // 创建/打开共享内存文件 0666 644 R/W permissions
        LOG_DEBUG() << "shm_name_ IS " << shm_name_.c_str() << ", shm_fd is: " << shm_fd;
        // exit_on_error(shm_fd < 0, "shm_open failed!");
        if (shm_fd < 0)
        {
            if (EEXIST == errno)
            {
                LOG_DEBUG() << "shm already exist, open only.";
                return OpenOnly();
            }
            else
            {
                LOG_ERROR() << "create shm failed, error: " << strerror(errno);
                return false;
            }
        }

        int ret = ftruncate(shm_fd, shm_size); // 截断共享文件大小
        if (ret < 0)
        {
            LOG_ERROR() << "ftruncate failed: " << strerror(errno);
            close(shm_fd);
            return false;
        }

        // attach managed_shm_
        managed_shm_ = mmap(nullptr, shm_size, PROT_WRITE | PROT_READ, MAP_SHARED, shm_fd, 0); // 将共享内存文件进行内存映射
        if (managed_shm_ == MAP_FAILED)
        {
            LOG_ERROR() << "attach shm failed:" << strerror(errno);
            close(shm_fd);
            shm_unlink(shm_name_.c_str());
            return false;
        }

        close(shm_fd);

        // create field state_
        state_ = new (managed_shm_) State(MESSAGE_SIZE_16K);
        if (state_ == nullptr)
        {
            LOG_ERROR() << "create state failed.";
            munmap(managed_shm_, shm_size);
            managed_shm_ = nullptr;
            shm_unlink(shm_name_.c_str());
            return false;
        }

        // _shm_mutex = (SHM_MUTEX *)managed_shm_; // 获取共享内存锁
        slice_ = new (static_cast<char *>(managed_shm_) + sizeof(State)) ShmSlice(msg_size, block_count, true);
        if (slice_ == nullptr)
        {
            LOG_ERROR() << "create blocks failed.";
            state_->~State();
            state_ = nullptr;
            munmap(managed_shm_, shm_size);
            managed_shm_ = nullptr;
            shm_unlink(shm_name_.c_str());
            return false;
        }
        state_->IncreaseReferenceCounts();
        init_ = true;
        LOG_DEBUG() << "create true." << shm_fd << ", slice_" << slice_;
        return true;
    }

    bool OpenOnly()
    {
        if (init_)
        {
            return true;
        }
        // get managed_shm_
        int fd = shm_open(shm_name_.c_str(), O_RDWR, 0666);
        if (fd == -1)
        {
            LOG_ERROR() << "get shm failed: " << strerror(errno);
            return false;
        }
        struct stat file_attr;
        if (fstat(fd, &file_attr) < 0)
        {
            LOG_ERROR() << "fstat failed: " << strerror(errno);
            close(fd);
            return false;
        }
        // attach managed_shm_
        managed_shm_ = mmap(nullptr, file_attr.st_size, PROT_READ | PROT_WRITE,
                            MAP_SHARED, fd, 0);
        if (managed_shm_ == MAP_FAILED)
        {
            LOG_ERROR() << "attach shm failed: " << strerror(errno);
            close(fd);
            return false;
        }
        close(fd);
        // get field state_
        state_ = reinterpret_cast<State *>(managed_shm_);
        if (state_ == nullptr)
        {
            LOG_ERROR() << "get state failed.";
            munmap(managed_shm_, file_attr.st_size);
            managed_shm_ = nullptr;
            return false;
        }

        slice_ = reinterpret_cast<ShmSlice *>((static_cast<char *>(managed_shm_) + sizeof(State)));
        // slice_ = new (static_cast<char *>(managed_shm_) + sizeof(State)) ShmSlice(m_msgSize, m_blockCount, false);
        if (slice_ == nullptr)
        {
            LOG_ERROR() << "create blocks failed.";
            state_->~State();
            state_ = nullptr;
            munmap(managed_shm_, shm_size);
            managed_shm_ = nullptr;
            shm_unlink(shm_name_.c_str());
            return false;
        }

        state_->IncreaseReferenceCounts();
        init_ = true;
        LOG_DEBUG() << "open only true." << fd << ", slice_" << slice_;
        return true;
    }

    bool Destroy()
    {
        if (!init_)
        {
            return true;
        }
        init_ = false;
        try
        {
            state_->DecreaseReferenceCounts();
            uint32_t reference_counts = state_->reference_counts();
            LOG_DEBUG() << "reference_counts." << reference_counts;

            if (reference_counts == 0)
            {
                return Remove();
            }
        }
        catch (...)
        {
            LOG_ERROR() << "exception.";
            return false;
        }
        LOG_DEBUG() << "destroy.";
        return true;
    }

    bool Remove()
    {
        if (slice_)
        {
            slice_->Destroy();
            slice_ = nullptr;
        }
        if (shm_unlink(shm_name_.c_str()) < 0)
        {
            LOG_ERROR() << "shm_unlink failed: " << strerror(errno);
            return false;
        }
        return true;
    }

    void Reset()
    {
        state_ = nullptr;
        if (slice_)
        {
            slice_->Destroy();
            slice_ = nullptr;
        }
        if (managed_shm_ != nullptr)
        {
            munmap(managed_shm_, shm_size);
            managed_shm_ = nullptr;
            return;
        }
    }

    void *slice_buf() const
    {
        assert(slice_ != nullptr);
        return slice_;
    }
};

END_NS_ZF_FRAMEWORK // zf::framework

#endif