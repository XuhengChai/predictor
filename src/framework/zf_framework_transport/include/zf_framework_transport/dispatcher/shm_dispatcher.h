

#ifndef ZF_TRANSPORT_DISPATCHER_SHM_DISPATCHER_H_
#define ZF_TRANSPORT_DISPATCHER_SHM_DISPATCHER_H_

#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "zf_framework_transport/shm/notifier_factory.h"
#include "zf_framework_transport/shm/segment_factory.h"
#include "zf_framework_transport/dispatcher/dispatcher.h"

#include "zf_global/common/zf_global_macros.h"
#include "zf_global/util/base_atomic_rw_lock.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

    class ShmDispatcher : public Dispatcher
{
public:
  using ShmDispatcherPtr = ShmDispatcher *;

  virtual ~ShmDispatcher();

  void Shutdown() override;

  template <typename MessageT>
  void AddListener(const RoleAttributes &self_attr,
                   const MessageListener<MessageT> &listener,
                   const RoleAttributes &opposite_attr = RoleAttributes());

private:
  void AddSegment(const RoleAttributes &self_attr);
  void ReadMessage(uint64_t channel_id, uint32_t block_index);
  void OnMessage(uint64_t channel_id, const std::shared_ptr<ReadableBlock> &rb,
                 const MessageInfo &msg_info);
  void ThreadFunc();
  bool Init();

  uint64_t host_id_;
  std::string host_ip_;
  std::unordered_map<uint64_t, SegmentPtr> segments_; // key: channel_id
  std::unordered_map<uint64_t, uint32_t> previous_indexes_;
  AtomicRWLock segments_lock_;
  std::thread thread_;
  NotifierPtr notifier_;

  DECLARE_SINGLETON(ShmDispatcher)
};

template <typename MessageT>
void ShmDispatcher::AddListener(const RoleAttributes &self_attr,
                                const MessageListener<MessageT> &listener,
                                const RoleAttributes &opposite_attr)
{
  // FIXME: make it more clean
  auto listener_adapter = [listener](const std::shared_ptr<ReadableBlock> &rb,
                                     const MessageInfo &msg_info)
  {
    auto msg = std::make_shared<MessageT>();
    // if (!MsgBase::ParseFromArray(
    //         rb->buf, static_cast<int>(rb->block->msg_size()), msg.get()))
    if (!msg->ParseFromArray(
            rb->buf, static_cast<int>(rb->block->msg_size())))
    {
      return;
    }
    listener(msg, msg_info);
  };
  Dispatcher::AddListener<ReadableBlock>(self_attr,
                                         listener_adapter, opposite_attr);
  AddSegment(self_attr);
}

END_NS_ZF_FRAMEWORK

#endif // ZF_TRANSPORT_DISPATCHER_SHM_DISPATCHER_H_
