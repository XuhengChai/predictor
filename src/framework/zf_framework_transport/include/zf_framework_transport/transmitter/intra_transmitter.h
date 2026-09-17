

#ifndef ZF_TRANSPORT_TRANSMITTER_INTRA_TRANSMITTER_H_
#define ZF_TRANSPORT_TRANSMITTER_INTRA_TRANSMITTER_H_

#include <memory>
#include <string>

#include "cyber/common/log.h"
#include "cyber/transport/dispatcher/intra_dispatcher.h"
#include "cyber/transport/transmitter/transmitter.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

template <typename M>
class IntraTransmitter : public Transmitter<M> {
 public:
  using MessagePtr = std::shared_ptr<M>;

  explicit IntraTransmitter(const RoleAttributes& attr);
  virtual ~IntraTransmitter();

  void Enable() override;
  void Disable() override;

  bool Transmit(const MessagePtr& msg, const MessageInfo& msg_info) override;

 private:
  uint64_t channel_id_;
  IntraDispatcherPtr dispatcher_;
};

template <typename M>
IntraTransmitter<M>::IntraTransmitter(const RoleAttributes& attr)
    : Transmitter<M>(attr),
      channel_id_(attr.GetChannelId()),
      dispatcher_(nullptr) {}

template <typename M>
IntraTransmitter<M>::~IntraTransmitter() {
  Disable();
}

template <typename M>
void IntraTransmitter<M>::Enable() {
  if (!this->enabled_) {
    dispatcher_ = IntraDispatcher::Instance();
    this->enabled_ = true;
  }
}

template <typename M>
void IntraTransmitter<M>::Disable() {
  if (this->enabled_) {
    dispatcher_ = nullptr;
    this->enabled_ = false;
  }
}

template <typename M>
bool IntraTransmitter<M>::Transmit(const MessagePtr& msg,
                                   const MessageInfo& msg_info) {
  if (!this->enabled_) {
    LOG_DEBUG() << "not enable.";
    return false;
  }

  dispatcher_->OnMessage(channel_id_, msg, msg_info);
  return true;
}

END_NS_ZF_FRAMEWORK

#endif  // ZF_TRANSPORT_TRANSMITTER_INTRA_TRANSMITTER_H_
