

#ifndef ZF_TRANSPORT_DISPATCHER_DISPATCHER_H_
#define ZF_TRANSPORT_DISPATCHER_DISPATCHER_H_

#include <atomic>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "zf_framework_transport/message/message_info.h"
#include "zf_framework_transport/message/listener_handler.h"

#include "zf_global/util/base_atomic_rw_lock.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

    class Dispatcher;
using DispatcherPtr = std::shared_ptr<Dispatcher>;

template <typename MessageT>
using MessageListener =
    std::function<void(const std::shared_ptr<MessageT> &, const MessageInfo &)>;

class Dispatcher
{
public:
  Dispatcher();
  virtual ~Dispatcher();

  virtual void Shutdown();

  template <typename MessageT>
  void AddListener(const RoleAttributes &self_attr,
                   const MessageListener<MessageT> &listener,
                   const RoleAttributes &opposite_attr = RoleAttributes());

  template <typename MessageT>
  void RemoveListener(const RoleAttributes &self_attr,
                      const RoleAttributes &opposite_attr = RoleAttributes());

  bool HasChannel(uint64_t channel_id);

protected:
  std::atomic<bool> is_shutdown_;
  // key: channel_id of message
  std::unordered_map<uint64_t, ListenerHandlerBasePtr> msg_listeners_;
  NS_ZF::AtomicRWLock rw_lock_;
};

template <typename MessageT>
void Dispatcher::AddListener(const RoleAttributes &self_attr,
                             const MessageListener<MessageT> &listener,
                             const RoleAttributes &opposite_attr)
{
  if (is_shutdown_.load())
  {
    return;
  }
  std::shared_ptr<ListenerHandler<MessageT>> handler;
  // ListenerHandlerBasePtr handler_base;
  uint64_t channel_id = self_attr.GetChannelId();
  if (msg_listeners_.find(channel_id) == msg_listeners_.end())
  {
    LOG_DEBUG() << "new reader for channel:"
                // << GlobalData::GetChannelById(channel_id);
                << channel_id;
    handler.reset(new ListenerHandler<MessageT>());
    msg_listeners_[channel_id] = handler;
  }
  else
  {
    ListenerHandlerBasePtr handler_base = msg_listeners_[channel_id];
    handler = std::dynamic_pointer_cast<ListenerHandler<MessageT>>(handler_base);
  }
  if (handler == nullptr)
  {
    LOG_ERROR() << "please ensure that readers with the same channel["
                << self_attr.GetChannelName()
                << "] in the same process have the same message type";
    return;
  }

  if (opposite_attr.IsEmpty())
  {
    handler->Connect(self_attr.GetId(), listener);
    return;
  }
  handler->Connect(self_attr.GetId(), opposite_attr.GetId(), listener);
  // LOG_DEBUG() << "Connect:" << self_attr.GetId() << "Connect:" << opposite_attr.GetId();
}

template <typename MessageT>
void Dispatcher::RemoveListener(const RoleAttributes &self_attr,
                                const RoleAttributes &opposite_attr)
{
  if (is_shutdown_.load())
  {
    return;
  }
  uint64_t channel_id = self_attr.GetChannelId();
  if (msg_listeners_.find(channel_id) == msg_listeners_.end())
  {
    return;
  }
  ListenerHandlerBasePtr handler_base = msg_listeners_[channel_id];
  if (opposite_attr.IsEmpty())
  {
    handler_base->Disconnect(self_attr.GetId());
    return;
  }
  handler_base->Disconnect(self_attr.GetId(), opposite_attr.GetId());
}

END_NS_ZF_FRAMEWORK

#endif // ZF_TRANSPORT_DISPATCHER_DISPATCHER_H_
