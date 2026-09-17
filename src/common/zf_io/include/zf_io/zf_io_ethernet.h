#ifndef ZF_IO_ETHERNET_H_
#define ZF_IO_ETHERNET_H_

#include <sstream>
#include <iomanip>
#include <atomic> // for atomic

#include "zf_io/zf_io_session.h"
#include "zf_global/common/zf_global_error_code.h"
#include "zf_global/in/zf_framework_global.h"
#include "zf_global/in/zf_io_struct.h"


BEGIN_NS_ZF_FRAMEWORK  // zf::framework
    class MsgBase;

template <typename M>
class Transmitter;

template <typename M>
class Receiver;

using TransmitterPtr = std::shared_ptr<Transmitter<MsgBase>>;
using ReceiverPtr = std::shared_ptr<Receiver<MsgBase>>;

END_NS_ZF_FRAMEWORK

    BEGIN_NS_ZF_DRIVER_IO

    class EthernetClient : public Session
{
public:
  EthernetClient(/* args */);
  ~EthernetClient();
  void InitTransmit();
  void InitReceiver(){};
  NS_ZF::ErrorCode Open(const char *iface = "eth4");
  void LoopReceive();
  void Shutdown();
  bool Send(CAN_HDR candata);

private:
  bool ParserBuffer(char *);
  std::atomic<bool> m_bIsShutdown;
  MAC_FRM_HDR m_frameMacSender;
  // std::shared_ptr<NS_ZF_FRAMEWORK::PosixSegmentMutex> m_segment;
  NS_ZF_FRAMEWORK::TransmitterPtr m_transmitterEthCAN;
  NS_ZF_FRAMEWORK::ReceiverPtr m_recvEthCAN;

  template <typename T>
  static std::string Int2Hex(T val, size_t width = sizeof(T) * 2)
  {
    std::stringstream ss;
    ss << std::uppercase << std::setfill('0') << std::setw(width) << std::hex << (val | 0);
    return ss.str();
  }
  template <typename T>
  static std::string PGNFromCanId(T id, size_t width = sizeof(T) * 2)
  {
    std::stringstream ssHex;
    ssHex << std::uppercase << std::setfill('0') << std::setw(width) << std::hex << (id | 0);
    if (ssHex.str().length() >= 6)
    {
      // return ssHex.str().substr(2, 4);
      return ssHex.str();
    }
    else
    {
      return ssHex.str();
    }
  }
  std::string CanFrameString(CAN_HDR *buf) const;
};

END_NS_ZF_DRIVER_IO

#endif // ZF_IO_ETHERNET_H_
