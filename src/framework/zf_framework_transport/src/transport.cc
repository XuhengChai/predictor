

#include "zf_framework_transport/transport.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

Transport::Transport()
{
  notifier_ = NotifierFactory::CreateNotifier();
}

Transport::~Transport() { Shutdown(); }

void Transport::Shutdown()
{
  if (is_shutdown_.exchange(true))
  {
    return;
  }
  LOG_DEBUG() << "Transport::Shutdown";
  ShmDispatcher::Instance().Shutdown();
  notifier_->Shutdown();
}

END_NS_ZF_FRAMEWORK
