
#include "zf_framework_transport/dispatcher/shm_dispatcher.h"

#include "zf_framework_transport/shm/condition_notifier.h"
#include "zf_global/common/zf_global_data.h"

BEGIN_NS_ZF_FRAMEWORK  // zf::framework

ShmDispatcher::ShmDispatcher()
    : host_id_(0) {
  Init();
}

ShmDispatcher::~ShmDispatcher() { Shutdown(); }

void ShmDispatcher::Shutdown() {
  if (is_shutdown_.exchange(true)) {
    return;
  }
  if (thread_.joinable()) {
    thread_.join();
  }
  {
    ReadLockGuard<AtomicRWLock> lock(segments_lock_);
    segments_.clear();
  }
}

void ShmDispatcher::AddSegment(const RoleAttributes &self_attr) {
  uint64_t channel_id = self_attr.GetChannelId();
  WriteLockGuard<AtomicRWLock> lock(segments_lock_);
  if (segments_.count(channel_id)) {
    return;
  }
  auto segment = SegmentFactory::CreateSegment(channel_id);
  segments_[channel_id] = segment;
  previous_indexes_[channel_id] = UINT32_MAX;
  LOG_DEBUG() << "add Segment: "
              << GlobalData::Instance().GetChannelById(channel_id);
}

void ShmDispatcher::ReadMessage(uint64_t channel_id, uint32_t block_index) {
  // LOG_DEBUG() << "Reading sharedmem message: "
  //             << GlobalData::Instance().GetChannelById(channel_id)
  //             << " from block: " << block_index;
  auto rb = std::make_shared<ReadableBlock>();
  rb->index = block_index;
  if (!segments_[channel_id]->AcquireBlockToRead(rb.get())) {
    LOG_WARN() << "fail to acquire block, channel: "
               << GlobalData::Instance().GetChannelById(channel_id)
               << " index: " << block_index;
    return;
  }

  MessageInfo msg_info;
  const char *msg_info_addr =
      reinterpret_cast<char *>(rb->buf) + rb->block->msg_size();

  if (msg_info.DeserializeFrom(msg_info_addr, rb->block->msg_info_size())) {
    OnMessage(channel_id, rb, msg_info);
  } else {
    LOG_ERROR() << "error msg info of channel:"
                << GlobalData::Instance().GetChannelById(channel_id);
  }
  segments_[channel_id]->ReleaseReadBlock(*rb);
}

void ShmDispatcher::OnMessage(uint64_t channel_id,
                              const std::shared_ptr<ReadableBlock> &rb,
                              const MessageInfo &msg_info) {
  if (is_shutdown_.load()) {
    return;
  }
  if (msg_listeners_.find(channel_id) == msg_listeners_.end()) {
    LOG_ERROR() << "Cannot find "
                << GlobalData::Instance().GetChannelById(channel_id)
                << "'s handler.";
    return;
  }
  ListenerHandlerBasePtr handler_base = msg_listeners_[channel_id];
  auto handler =
      std::dynamic_pointer_cast<ListenerHandler<ReadableBlock>>(handler_base);
  handler->Run(rb, msg_info);
}

void ShmDispatcher::ThreadFunc() {
  ReadableInfo readable_info;
  while (!is_shutdown_.load()) {
    if (!notifier_->Listen(100, &readable_info)) {
      LOG_DEBUG() << "listen failed.";
      continue;
    }

    if (readable_info.host_id() != host_id_) {
      LOG_DEBUG() << "shm readable info from other host.";
      continue;
    }

    uint64_t channel_id = readable_info.channel_id();
    uint32_t block_index = readable_info.block_index();
    {
      ReadLockGuard<AtomicRWLock> lock(segments_lock_);
      if (segments_.count(channel_id) == 0) {
        // LOG_WARN() << "no segments_ of channel:"
        //            << GlobalData::Instance().GetChannelById(channel_id);
        continue;
      }
      // check block index
      if (previous_indexes_.count(channel_id) == 0) {
        previous_indexes_[channel_id] = UINT32_MAX;
      }
      uint32_t &previous_index = previous_indexes_[channel_id];
      if (block_index != 0 && previous_index != UINT32_MAX) {
        if (block_index == previous_index) {
          LOG_DEBUG() << "Receive SAME index " << block_index << " of channel ";
        } else if (block_index < previous_index) {
          LOG_DEBUG() << "Receive PREVIOUS message. last: " << previous_index
                      << ", now: " << block_index;
        } else if (block_index - previous_index > 1) {
          // LOG_DEBUG() << "Receive JUMP message. last: " << previous_index
          //             << ", now: " << block_index;
        }
      }
      previous_index = block_index;

      ReadMessage(channel_id, block_index);
    }
  }
}

bool ShmDispatcher::Init() {
  host_id_ = NS_ZF_STRING_UTIL::Hash(GlobalData::Instance().HostIp());
  notifier_ = NotifierFactory::CreateNotifier();
  thread_ = std::thread(&ShmDispatcher::ThreadFunc, this);
  // scheduler::Instance()->SetInnerThreadAttr("shm_disp", &thread_);
  return true;
}

END_NS_ZF_FRAMEWORK
