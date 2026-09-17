

#ifndef ZF_TRANSPORT_RECEIVER_SHM_RECEIVER_H_
#define ZF_TRANSPORT_RECEIVER_SHM_RECEIVER_H_

#include <functional>

#include "zf_framework_transport/dispatcher/shm_dispatcher.h"
#include "zf_framework_transport/receiver/receiver.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

template <typename M>
class ShmReceiver : public Receiver<M>
{
public:
  ShmReceiver(const RoleAttributes &attr,
              const typename Receiver<M>::MessageListener &msg_listener);
  virtual ~ShmReceiver();
  void Enable(const RoleAttributes &opposite_attr = RoleAttributes()) override;
  void Disable(const RoleAttributes &opposite_attr = RoleAttributes()) override;

private:
  ShmDispatcher *dispatcher_;
};

template <typename M>
ShmReceiver<M>::ShmReceiver(
    const RoleAttributes &attr,
    const typename Receiver<M>::MessageListener &msg_listener)
    : Receiver<M>(attr, msg_listener)
{
}

template <typename M>
ShmReceiver<M>::~ShmReceiver()
{
  Disable();
}

template <typename M>
void ShmReceiver<M>::Enable(const RoleAttributes &opposite_attr)
{
  ShmDispatcher::Instance().AddListener<M>(
      this->attr_,
      std::bind(&ShmReceiver<M>::OnNewMessage, this, std::placeholders::_1,
                std::placeholders::_2),
      opposite_attr);
}

template <typename M>
void ShmReceiver<M>::Disable(const RoleAttributes &opposite_attr)
{
  ShmDispatcher::Instance().RemoveListener<M>(this->attr_, opposite_attr);
}

END_NS_ZF_FRAMEWORK

#endif // ZF_TRANSPORT_RECEIVER_SHM_RECEIVER_H_
