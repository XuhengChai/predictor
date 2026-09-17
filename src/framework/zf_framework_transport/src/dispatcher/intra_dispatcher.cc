

#include "cyber/transport/dispatcher/intra_dispatcher.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

IntraDispatcher::IntraDispatcher() { chain_.reset(new ChannelChain()); }

IntraDispatcher::~IntraDispatcher() {}

END_NS_ZF_FRAMEWORK
