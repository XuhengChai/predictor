

#ifndef ZF_TRANSPORT_TRANSPORT_H_
#define ZF_TRANSPORT_TRANSPORT_H_

#include <atomic>
#include <memory>
#include <string>

#include "zf_framework_transport/receiver/receiver.h"
#include "zf_framework_transport/receiver/shm_receiver.h"
#include "zf_framework_transport/shm/notifier_factory.h"
#include "zf_framework_transport/transmitter/transmitter.h"
#include "zf_framework_transport/transmitter/shm_transmitter.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

enum class OptionalMode {
  INTRA = 0,
  SHM = 1,
  RTPS = 2,
  HYBRID = 3,
};

class Transport
{
public:
  virtual ~Transport();
  void Shutdown();
  template <typename M>
  auto CreateTransmitter(const RoleAttributes &attr,
                         const OptionalMode &mode = OptionalMode::HYBRID) ->
      typename std::shared_ptr<Transmitter<M>>;

  template <typename M>
  auto CreateReceiver(const RoleAttributes &attr,
                      const typename Receiver<M>::MessageListener &msg_listener,
                      const OptionalMode &mode = OptionalMode::HYBRID) ->
      typename std::shared_ptr<Receiver<M>>;

private:
  std::atomic<bool> is_shutdown_ = {false};
  NotifierPtr notifier_ = nullptr;
  DECLARE_SINGLETON(Transport)
};

template <typename M>
auto Transport::CreateTransmitter(const RoleAttributes &attr,
                                  const OptionalMode &mode) ->
    typename std::shared_ptr<Transmitter<M>>
{
  if (is_shutdown_.load())
  {
    LOG_INFO() << "transport has been shut down.";
    return nullptr;
  }

  // std::shared_ptr<Transmitter<M>> transmitter = nullptr;
  // RoleAttributes modified_attr = attr;
  // if (!modified_attr.has_qos_profile())
  // {
  //   modified_attr.mutable_qos_profile()->CopyFrom(
  //       QosProfileConf::QOS_PROFILE_DEFAULT);
  // }

  // switch (mode)
  // {
  // case OptionalMode::INTRA:
  //   transmitter = std::make_shared<IntraTransmitter<M>>(modified_attr);
  //   break;

  // case OptionalMode::SHM:
  //   transmitter = std::make_shared<ShmTransmitter<M>>(modified_attr);
  //   break;

  // case OptionalMode::RTPS:
  //   transmitter =
  //       std::make_shared<RtpsTransmitter<M>>(modified_attr, participant());
  //   break;

  // default:
  //   transmitter =
  //       std::make_shared<HybridTransmitter<M>>(modified_attr, participant());
  //   break;
  // }
  std::shared_ptr<Transmitter<M>> transmitter = std::make_shared<ShmTransmitter<M>>(attr);
  RETURN_VAL_IF_NULL(transmitter, nullptr);
  if (mode != OptionalMode::HYBRID)
  {
    transmitter->Enable();
  }
  return transmitter;
}

template <typename M>
auto Transport::CreateReceiver(
    const RoleAttributes &attr,
    const typename Receiver<M>::MessageListener &msg_listener,
    const OptionalMode &mode) -> typename std::shared_ptr<Receiver<M>>
{
  if (is_shutdown_.load())
  {
    LOG_INFO() << "transport has been shut down.";
    return nullptr;
  }

  // std::shared_ptr<Receiver<M>> receiver = nullptr;
  // RoleAttributes modified_attr = attr;
  // if (!modified_attr.has_qos_profile())
  // {
  //   modified_attr.mutable_qos_profile()->CopyFrom(
  //       QosProfileConf::QOS_PROFILE_DEFAULT);
  // }

  // switch (mode)
  // {
  // case OptionalMode::INTRA:
  //   receiver =
  //       std::make_shared<IntraReceiver<M>>(modified_attr, msg_listener);
  //   break;

  // case OptionalMode::SHM:
  //   receiver = std::make_shared<ShmReceiver<M>>(modified_attr, msg_listener);
  //   break;

  // case OptionalMode::RTPS:
  //   receiver = std::make_shared<RtpsReceiver<M>>(modified_attr, msg_listener);
  //   break;

  // default:
  //   receiver = std::make_shared<HybridReceiver<M>>(
  //       modified_attr, msg_listener, participant());
  //   break;
  // }
  std::shared_ptr<Receiver<M>> receiver = std::make_shared<ShmReceiver<M>>(attr, msg_listener);
  RETURN_VAL_IF_NULL(receiver, nullptr);
  if (mode != OptionalMode::HYBRID)
  {
    receiver->Enable();
  }
  return receiver;
}

END_NS_ZF_FRAMEWORK

#endif // ZF_TRANSPORT_TRANSPORT_H_
