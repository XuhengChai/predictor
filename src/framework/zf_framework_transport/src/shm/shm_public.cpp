#include "zf_framework_transport/shm/shm_public.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

PosixSegmentMutex::PosixSegmentMutex(std::string path)
{
    // shm_name_ = std::to_string(channel_id);
    shm_name_ = std::move(path);
    SetTimeout();
}

PosixSegmentMutex::~PosixSegmentMutex()
{
    Destroy();
}

END_NS_ZF_FRAMEWORK // zf::framework
