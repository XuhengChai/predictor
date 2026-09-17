#ifndef ZF_TRANSPORT_MESSAGE_LISTENER_HANDLER_H_
#define ZF_TRANSPORT_MESSAGE_LISTENER_HANDLER_H_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "zf_framework_transport/message/message_info.h"
#include "zf_framework_transport/message/message_base.h"

#include "zf_global/util/base_atomic_rw_lock.h"
#include "zf_global/util/base_sig_slot.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

    class ListenerHandlerBase;
using ListenerHandlerBasePtr = std::shared_ptr<ListenerHandlerBase>;

class ListenerHandlerBase
{
public:
  ListenerHandlerBase() {}
  virtual ~ListenerHandlerBase() {}
  // virtual void Connect(uint64_t self_id) = 0;
  // virtual void Connect(uint64_t self_id, uint64_t oppo_id) = 0;
  virtual void Disconnect(uint64_t self_id) = 0;
  virtual void Disconnect(uint64_t self_id, uint64_t oppo_id) = 0;
  inline bool IsRawMessage() const { return is_raw_message_; }
  // virtual void RunFromString(const std::string &str,
  //                            const MessageInfo &msg_info) = 0;

protected:
  bool is_raw_message_ = false;
};

template <typename MessageT>
class ListenerHandler : public ListenerHandlerBase
{
public:
  using Message = std::shared_ptr<MessageT>;
  using MessageSignal = NS_ZF::Signal<const Message &, const MessageInfo &>;

  using Listener = std::function<void(const Message &, const MessageInfo &)>;
  using MessageConnection =
      NS_ZF::Connection<const Message &, const MessageInfo &>;
  using ConnectionMap = std::unordered_map<uint64_t, MessageConnection>;

  ListenerHandler() {}
  virtual ~ListenerHandler() {}

  void Connect(uint64_t self_id, const Listener &listener);
  void Connect(uint64_t self_id, uint64_t oppo_id, const Listener &listener); // opposite id

  void Disconnect(uint64_t self_id) override;
  void Disconnect(uint64_t self_id, uint64_t oppo_id) override;

  void Run(const Message &msg, const MessageInfo &msg_info);
  // void RunFromString(const std::string &str,
  //                    const MessageInfo &msg_info) override;

private:
  using SignalPtr = std::shared_ptr<MessageSignal>;
  using MessageSignalMap = std::unordered_map<uint64_t, SignalPtr>;
  // used for self_id
  MessageSignal signal_;
  ConnectionMap signal_conns_; // key: self_id

  // used for self_id and opposite_id
  MessageSignalMap signals_; // key: oppo_id
  // key: oppo_id
  std::unordered_map<uint64_t, ConnectionMap> signals_conns_;

  NS_ZF::AtomicRWLock rw_lock_;
};

template <typename MessageT>
void ListenerHandler<MessageT>::Connect(uint64_t self_id,
                                        const Listener &listener)
{
  auto connection = signal_.Connect(listener);
  if (!connection.IsConnected())
  {
    return;
  }

  NS_ZF::WriteLockGuard<NS_ZF::AtomicRWLock> lock(rw_lock_);
  signal_conns_[self_id] = connection;
}

template <typename MessageT>
void ListenerHandler<MessageT>::Connect(uint64_t self_id, uint64_t oppo_id,
                                        const Listener &listener)
{
  NS_ZF::WriteLockGuard<NS_ZF::AtomicRWLock> lock(rw_lock_);
  if (signals_.find(oppo_id) == signals_.end())
  {
    signals_[oppo_id] = std::make_shared<MessageSignal>();
  }

  auto connection = signals_[oppo_id]->Connect(listener);
  if (!connection.IsConnected())
  {
    LOG_WARN() << oppo_id << " " << self_id << " connect failed!";
    return;
  }

  if (signals_conns_.find(oppo_id) == signals_conns_.end())
  {
    signals_conns_[oppo_id] = ConnectionMap();
  }

  signals_conns_[oppo_id][self_id] = connection;
}

template <typename MessageT>
void ListenerHandler<MessageT>::Disconnect(uint64_t self_id)
{
  NS_ZF::WriteLockGuard<NS_ZF::AtomicRWLock> lock(rw_lock_);
  if (signal_conns_.find(self_id) == signal_conns_.end())
  {
    return;
  }

  signal_conns_[self_id].Disconnect();
  signal_conns_.erase(self_id);
}

template <typename MessageT>
void ListenerHandler<MessageT>::Disconnect(uint64_t self_id, uint64_t oppo_id)
{
  NS_ZF::WriteLockGuard<NS_ZF::AtomicRWLock> lock(rw_lock_);
  if (signals_conns_.find(oppo_id) == signals_conns_.end())
  {
    return;
  }

  if (signals_conns_[oppo_id].find(self_id) == signals_conns_[oppo_id].end())
  {
    return;
  }

  signals_conns_[oppo_id][self_id].Disconnect();
  signals_conns_[oppo_id].erase(self_id);
}

template <typename MessageT>
void ListenerHandler<MessageT>::Run(const Message &msg,
                                    const MessageInfo &msg_info)
{
  signal_(msg, msg_info);
  uint64_t oppo_id = msg_info.sender_id().HashValue();
  NS_ZF::ReadLockGuard<NS_ZF::AtomicRWLock> lock(rw_lock_);
  if (signals_.find(oppo_id) == signals_.end())
  {
    return;
  }

  (*signals_[oppo_id])(msg, msg_info);
}

// template <typename MessageT>
// void ListenerHandler<MessageT>::RunFromString(const std::string &str,
//                                               const MessageInfo &msg_info)
// {
//   auto msg = std::make_shared<MessageT>();
//   // if (MsgBase::ParseFromArray(a, static_cast<int>(str.size()), msg.get()))
//   // auto msg = std::make_shared<MsgBase>();
//   if (!msg->ParseFromArray(str.data(), static_cast<int>(str.size())))
//   {
//     Run(msg, msg_info);
//   }
//   else
//   {
//     LOG_WARN() << "Failed to parse message. Content: " << str;
//   }
// }

END_NS_ZF_FRAMEWORK

#endif // ZF_TRANSPORT_MESSAGE_LISTENER_HANDLER_H_
