

#ifndef ZF_TRANSPORT_TRANSMITTER_SHM_TRANSMITTER_H_
#define ZF_TRANSPORT_TRANSMITTER_SHM_TRANSMITTER_H_

#include <cstring>
#include <iostream>
#include <memory>
#include <string>

#include "zf_framework_transport/shm/notifier_factory.h"
#include "zf_framework_transport/shm/readable_info.h"
#include "zf_framework_transport/shm/segment_factory.h"
#include "zf_framework_transport/transmitter/transmitter.h"
#include "zf_global/util/string_util.h"
#include "zf_global/common/zf_global_data.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

    template <typename M>
    class ShmTransmitter : public Transmitter<M>
{
public:
  using MessagePtr = std::shared_ptr<M>;

  explicit ShmTransmitter(const RoleAttributes &attr);
  virtual ~ShmTransmitter();

  void Enable() override;
  void Disable() override;

  bool Transmit(const MessagePtr &msg, const MessageInfo &msg_info) override;

private:
  bool Transmit(const M &msg, const MessageInfo &msg_info);

  SegmentPtr segment_;
  uint64_t channel_id_;
  uint64_t host_id_;
  NotifierPtr notifier_;
};

template <typename M>
ShmTransmitter<M>::ShmTransmitter(const RoleAttributes &attr)
    : Transmitter<M>(),
      segment_(nullptr),
      channel_id_(attr.GetChannelId()),
      notifier_(nullptr)
{
  host_id_ = NS_ZF_STRING_UTIL::Hash(attr.GetHostIp());
}

template <typename M>
ShmTransmitter<M>::~ShmTransmitter()
{
  Disable();
}

template <typename M>
void ShmTransmitter<M>::Enable()
{
  if (this->enabled_)
  {
    return;
  }

  segment_ = SegmentFactory::CreateSegment(channel_id_);
  notifier_ = NotifierFactory::CreateNotifier();
  this->enabled_ = true;
}

template <typename M>
void ShmTransmitter<M>::Disable()
{
  if (this->enabled_)
  {
    segment_ = nullptr;
    notifier_ = nullptr;
    this->enabled_ = false;
  }
}

template <typename M>
bool ShmTransmitter<M>::Transmit(const MessagePtr &msg,
                                 const MessageInfo &msg_info)
{
  return Transmit(*msg, msg_info);
}

template <typename M>
bool ShmTransmitter<M>::Transmit(const M &msg, const MessageInfo &msg_info)
{
  if (!this->enabled_)
  {
    LOG_DEBUG() << "not enable.";
    return false;
  }

  WritableBlock wb;
  // std::size_t msg_size = MsgBase::ByteSize(msg);
  std::size_t msg_size = msg.ByteSize();
  if (!segment_->AcquireBlockToWrite(msg_size, &wb))
  {
    LOG_ERROR() << "acquire block failed.";
    return false;
  }

  // LOG_DEBUG() << "block index: " << wb.index;
  // LOG_DEBUG() << "wb: " << &wb;
  // LOG_DEBUG() << "wb.buf: " << wb.buf;
  // if (!MsgBase::SerializeToArray(msg, wb.buf, static_cast<int>(msg_size)))
  if (!msg.SerializeToArray(wb.buf, static_cast<int>(msg_size)))
  {
    LOG_ERROR() << "serialize to array failed.";
    segment_->ReleaseWrittenBlock(wb);
    return false;
  }
  wb.block->set_msg_size(msg_size);
  // LOG_DEBUG() << "block set_msg_size: " << msg_size;

  char *msg_info_addr = reinterpret_cast<char *>(wb.buf) + msg_size;
  // LOG_DEBUG() << "msg_info_addr: " << msg_info_addr;

  if (!msg_info.SerializeTo(msg_info_addr, MessageInfo::kSize))
  {
    LOG_ERROR() << "serialize message info failed.";
    segment_->ReleaseWrittenBlock(wb);
    return false;
  }
  wb.block->set_msg_info_size(MessageInfo::kSize);
  // LOG_DEBUG() << "block set_msg_info_size: " << MessageInfo::kSize;

  segment_->ReleaseWrittenBlock(wb);

  ReadableInfo readable_info(host_id_, wb.index, channel_id_);

  // LOG_DEBUG() << "Writing sharedmem message: "
  //             << GlobalData::Instance().GetChannelById(channel_id_)
  //             << " to block: " << wb.index;
  return notifier_->Notify(readable_info);
}

END_NS_ZF_FRAMEWORK

#endif // ZF_TRANSPORT_TRANSMITTER_SHM_TRANSMITTER_H_
