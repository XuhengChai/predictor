#include "zf_hal_can_driver/client/can_client.h"
#include "zf_hal_can_driver/client/socket_can/can_filter.h"

/**
 * @namespace zf::driver::canbus
 */
BEGIN_NS_ZF_DRIVER_CANBUS

CanClient::CanClient()
    : m_filter(nullptr)
{}

CanClient::~CanClient() = default;
CanClient::CanClient(CanClient&& rhs) = default;
CanClient& CanClient::operator=(CanClient&& rhs) = default;

END_NS_ZF_DRIVER_CANBUS