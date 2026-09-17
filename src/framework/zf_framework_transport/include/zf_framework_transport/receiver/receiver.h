

#ifndef ZF_TRANSPORT_RECEIVER_RECEIVER_H_
#define ZF_TRANSPORT_RECEIVER_RECEIVER_H_

#include <functional>
#include <memory>

#include "zf_framework_transport/message/message_info.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

    template <typename M>
    class Receiver
{
public:
  using MessagePtr = std::shared_ptr<M>;
  using MessageListener = std::function<void(
      const MessagePtr &, const MessageInfo &, const RoleAttributes &)>;

  Receiver(const RoleAttributes &attr, const MessageListener &msg_listener);
  virtual ~Receiver();
  virtual void Enable(const RoleAttributes &opposite_attr = RoleAttributes()) = 0;
  virtual void Disable(const RoleAttributes &opposite_attr = RoleAttributes()) = 0;
  const Identity &id() const { return id_; }

protected:
  void OnNewMessage(const MessagePtr &msg, const MessageInfo &msg_info);
  MessageListener msg_listener_;
  RoleAttributes attr_;
  bool enabled_ = {};
  Identity id_;
};

template <typename M>
Receiver<M>::Receiver(const RoleAttributes &attr,
                      const MessageListener &msg_listener)
    : attr_(attr), msg_listener_(msg_listener)
{
}

template <typename M>
Receiver<M>::~Receiver() {}

template <typename M>
void Receiver<M>::OnNewMessage(const MessagePtr &msg,
                               const MessageInfo &msg_info)
{
  if (msg_listener_ != nullptr)
  {
    msg_listener_(msg, msg_info, attr_);
  }
}

END_NS_ZF_FRAMEWORK

#endif // ZF_TRANSPORT_RECEIVER_RECEIVER_H_
