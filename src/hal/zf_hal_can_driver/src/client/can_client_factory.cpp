#include "zf_hal_can_driver/client/can_client_factory.h"
#include "zf_hal_can_driver/client/socket_can/socket_can_client.h"
#include "zf_hal_can_driver/client/ethernet_can/ethernet_can_client.h"
#include "zf_hal_can_driver/client/fake_can/fake_can_client.h"
#include "zf_hal_can_driver/client/pcan/pcan_client.h"

/**
 * @namespace zf::driver::canbus
 */
BEGIN_NS_ZF_DRIVER_CANBUS

CanClientFactory::CanClientFactory() {}

void CanClientFactory::RegisterCanClients()
{
  LOG_INFO() << "CanClientFactory::RegisterCanClients";
  Register(CANCardParameter::CANCardBrand::FAKE_CAN,
           []() -> CanClient *
           { return new FakeCanClient(); });
  Register(CANCardParameter::CANCardBrand::SOCKET_CAN,
           []() -> CanClient *
           { return new SocketCanClient(); });
  Register(CANCardParameter::CANCardBrand::ETHERNET_CAN,
           []() -> CanClient *
           { return new EthernetClient(); });
  Register(CANCardParameter::CANCardBrand::PCAN,
           []() -> CanClient *
           { return new PCanClient(); });
}

std::unique_ptr<CanClient> CanClientFactory::CreateCANClient(
    const CANCardParameter &parameter)
{
  auto factory = CreateObject(parameter.brand);
  if (!factory)
  {
    LOG_ERROR() << "Failed to create CAN client with parameter: " << parameter.DebugString().c_str();
  }
  else if (!factory->Init(parameter))
  {
    LOG_ERROR() << "Failed to initialize CAN card with parameter: " << parameter.DebugString().c_str();
  }
  return factory;
}

END_NS_ZF_DRIVER_CANBUS
