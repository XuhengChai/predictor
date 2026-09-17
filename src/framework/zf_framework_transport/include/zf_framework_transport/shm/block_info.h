#ifndef ZF_TRANSPORT_SHM_DATA_H_
#define ZF_TRANSPORT_SHM_DATA_H_

#include <fcntl.h>
#include <unistd.h>
#include <memory>
#include <cstring>

#include "zf_global/in/zf_framework_global.h"
// #include "msg/zf_msg_warp_base.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

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
        if (!written_) //has been written
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



END_NS_ZF_FRAMEWORK

#endif // ZF_TRANSPORT_SHM_DATA_H_
