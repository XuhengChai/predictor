#include "zf_framework_transport/dispatcher/dispatcher.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

Dispatcher::Dispatcher() : is_shutdown_(false) {}

Dispatcher::~Dispatcher() { Shutdown(); }

void Dispatcher::Shutdown() {
  is_shutdown_.store(true);
  LOG_DEBUG() << "Shutdown";
}

bool Dispatcher::HasChannel(uint64_t channel_id) {
  return msg_listeners_.find(channel_id) != msg_listeners_.end();
}

END_NS_ZF_FRAMEWORK
