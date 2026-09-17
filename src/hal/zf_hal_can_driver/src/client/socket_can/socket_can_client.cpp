#include "zf_hal_can_driver/client/socket_can/can_filter.h"
#include "zf_hal_can_driver/client/socket_can/socket_can_client.h"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/sockios.h>

/**
 * @namespace zf::driver::canbus
 */
BEGIN_NS_ZF_DRIVER_CANBUS

#define CAN_ID_MASK 0x1FFFF800U // can_filter mask

timeval to_timeval(const std::chrono::nanoseconds timeout) noexcept
{
  const auto count = timeout.count();
  constexpr auto BILLION = 1'000'000'000LL;
  timeval c_timeout;
  c_timeout.tv_sec = static_cast<decltype(c_timeout.tv_sec)>(count / BILLION);
  c_timeout.tv_usec = static_cast<decltype(c_timeout.tv_usec)>((count % BILLION) / 1000LL);

  return c_timeout;
}

bool SocketCanClient::Init(const CANCardParameter &parameter)
{
  m_canParam = parameter;
  if (static_cast<uint8_t>(m_canParam.channelID) > (m_canParam.portsNum) || m_canParam.channelID < 0)
  {
    LOG_ERROR() << "Can port number [" << m_canParam.channelID << "] is out of range [0, "
                << m_canParam.portsNum << ") !";
    return false;
  }
  m_fdMode = m_canParam.enableFD ? MODE_CANFD_MTU : MODE_CAN_MTU;
  return true;
}

SocketCanClient::SocketCanClient()
{
  m_filter = std::make_unique<CanFilterList>();
}

SocketCanClient::~SocketCanClient()
{
  if (m_iDevHandler)
  {
    Close();
  }
}

ErrorCode SocketCanClient::Open()
{
  if (m_bIsOpened)
  {
    return ErrorCode::OK;
  }

  // open device
  // guss net is the device minor number, if one card is 0,1
  // if more than one card, when install driver u can specify the minior id
  // int32_t ret = canOpen(net, pCtx->mode, txbufsize, rxbufsize, 0, 0,
  // &m_iDevHandler);
  m_iDevHandler = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (m_iDevHandler < 0)
  {
    LOG_ERROR() << "open device error code [" << m_iDevHandler << "]: ";
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  // // // Make it non-blocking so we can use timeouts
  // // lint -e{9001} NOLINT I can't do anything about using this third party octal constant...
  // if (0 != fcntl(m_iDevHandler, F_SETFL, O_NONBLOCK))
  // {
  //   throw std::runtime_error{"Failed to set CAN socket to nonblocking"};
  //   LOG_ERROR() << "Failed to set CAN socket to nonblocking[" << m_iDevHandler << "]: ";
  //   return ErrorCode::CAN_CLIENT_ERROR_BASE;
  // }

  // init config and state
  int ret;

  // // 1. for non virtual busses, set receive message_id filter, ie white list
  // if (m_canParam.interface != CANCardParameter::VIRTUAL)
  // {
  //   // set a scope for each EID instead of a single filter rule for each EID
  //   struct can_filter filter;
  //   filter.can_id = 0x000;
  //   filter.can_mask = CAN_ID_MASK;
  //   ret = m_filter->SetCanFilter(m_iDevHandler, {filter});
  //   if (ret < 0)
  //   {
  //     LOG_ERROR() << "add receive msg id filter error code: " << ret;
  //     return ErrorCode::CAN_CLIENT_ERROR_BASE;
  //   }
  // }

  struct timeval tv = m_canParam.timestamp;
  // tv.tv_sec = m_canParam.sec;                             /* 30 Secs Timeout */
  // tv.tv_usec = m_canParam.timeout_ms * 1000; // Not init'ing this can cause strange errors???
  setsockopt(m_iDevHandler, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(struct timeval));

  std::string can_name;
  if (m_canParam.deviceName.empty())
  {
    std::string interface_prefix;
    if (m_canParam.interface == CANCardParameter::VIRTUAL)
    {
      interface_prefix = "vcan";
    }
    else if (m_canParam.interface == CANCardParameter::SLCAN)
    {
      interface_prefix = "slcan";
    }
    else
    { // default: CANCardParameter::NATIVE
      interface_prefix = "can";
    }
    can_name = interface_prefix + std::to_string(m_canParam.channelID);
  }
  else
  {
    can_name = m_canParam.deviceName;
  }

  // 3. Set up address/interface name
  struct ifreq ifr;
  // std::strncpy(ifr.ifr_name, can_name.c_str(), IFNAMSIZ);
  std::strncpy(ifr.ifr_name, can_name.c_str(), can_name.length() + 1U);
  if (ioctl(m_iDevHandler, SIOCGIFINDEX, &ifr) < 0) //== -1
  {
    LOG_ERROR() << "ioctl error";
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  // 4. bind socket to network interface
  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  ret = ::bind(m_iDevHandler, reinterpret_cast<struct sockaddr *>(&addr),
               sizeof(addr));

  if (ret < 0)
  {
    LOG_ERROR() << "bind socket to network interface error code: " << ret;
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  // 2. enable reception of can frames.
  const int32_t enable = m_canParam.enableFD ? 1 : 0;
  ret = ::setsockopt(m_iDevHandler, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable,
                     sizeof(enable));
  if (ret < 0)
  {
    LOG_ERROR() << "enable reception of can frame error code: " << ret;
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }
  LOG_INFO() << "can_name is---" << can_name << "---enable " << m_canParam.enableFD;

  m_bIsOpened = true;
  return ErrorCode::OK;
}

void SocketCanClient::Close()
{
  if (m_bIsOpened)
  {
    m_bIsOpened = false;

    int ret = close(m_iDevHandler);
    if (ret < 0)
    {
      LOG_ERROR() << "close error code:" << ret << ", " << GetErrorString(ret);
    }
    else
    {
      LOG_INFO() << "close socket can ok. port:" << m_canParam.channelID;
    }
  }
}

// Synchronous transmission of CAN messages
ErrorCode SocketCanClient::Send(const std::vector<CanFrame> &frames,
                                const int32_t &frame_num,
                                const std::chrono::nanoseconds &)
{
  CHECK_EQ(frames.size(), static_cast<size_t>(frame_num));

  if (!m_bIsOpened)
  {
    LOG_ERROR() << "SocketCanClient::Send: Nvidia socket can client has not been initiated! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
  }

  // if (Wait(timeout) != ErrorCode::OK)
  // {
  //   LOG_ERROR() << "SocketCanClient::Send: Timeout!";
  //   return ErrorCode::CAN_CLIENT_ERROR_TIMEOUT;
  // }
  if (m_fdMode == MODE_CANFD_MTU)
  {
    for (size_t i = 0; i < frames.size() && i < MAX_CAN_SEND_FRAME_LEN; ++i)
    {
      if (frames[i].len > static_cast<uint8_t>(CANBUS_MESSAGE_LENGTH_FD) || frames[i].len == 0)
      {
        LOG_ERROR() << "frames[" << i << "].len = " << frames[i].len
                    << ", which is not equal to can message data length (64).";
        return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
      }
      // memset(m_fdSendFrames[i].data, 0, sizeof(m_fdSendFrames[i].data));//sizeof(data)/sizeof(data[0])
      memset(m_fdSendFrames[i].data, 0, CANBUS_MESSAGE_LENGTH_FD); // dfeault is 0
      m_fdSendFrames[i].can_id = frames[i].id;
      m_fdSendFrames[i].len = frames[i].len;
      std::memcpy(m_fdSendFrames[i].data, frames[i].data, frames[i].len);

      // Synchronous transmission of CAN messages
      int ret = static_cast<int>(
          write(m_iDevHandler, &m_fdSendFrames[i], sizeof(m_fdSendFrames[i])));
      if (ret <= 0)
      {
        LOG_ERROR() <<frames[i].id << ", " << m_canParam.deviceName.c_str() << "send fd message failed, error code: " << ret;
        return ErrorCode::CAN_CLIENT_ERROR_BASE;
      }
      // LOG_DEBUG()  <<frames[i].id << ", " << m_canParam.deviceName.c_str() << "------send FD message success: " << ret;
    }
  }
  else
  {
    for (size_t i = 0; i < frames.size() && i < MAX_CAN_SEND_FRAME_LEN; ++i)
    {
      if (frames[i].len > static_cast<uint8_t>(CANBUS_MESSAGE_LENGTH) || frames[i].len == 0)
      {
        LOG_ERROR() << "frames[" << i << "].len = " << frames[i].len
                    << ", which is not equal to can message data length ("
                    << CANBUS_MESSAGE_LENGTH << ").";
        return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
      }
      send_frames_[i].can_id = frames[i].id;
      send_frames_[i].can_dlc = frames[i].len;
      std::memcpy(send_frames_[i].data, frames[i].data, frames[i].len);

      // Synchronous transmission of CAN messages
      int ret = static_cast<int>(
          write(m_iDevHandler, &send_frames_[i], sizeof(send_frames_[i])));
      if (ret <= 0)
      {
        LOG_ERROR() <<frames[i].id << ", " << m_canParam.deviceName.c_str() <<": send std message failed, error code: " << ret;
        return ErrorCode::CAN_CLIENT_ERROR_BASE;
      }
      // LOG_DEBUG() << m_canParam.deviceName.c_str() <<": -----send STD message success: " << ret;
    }
  }
  return ErrorCode::OK;
}

// buf size must be 8 bytes, every time, we receive only one frame
ErrorCode SocketCanClient::Receive(std::vector<CanFrame> *const frames,
                                   const int32_t &frame_num,
                                   const std::chrono::nanoseconds &)
{
  if (!m_bIsOpened)
  {
    LOG_ERROR() << "SocketCanClient::Receive: Nvidia socket can client is not init! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
  }

  // struct can_frame frame;
  // int nbytes = read(m_iDevHandler, &frame, sizeof(struct can_frame));
  // if (nbytes < 0)
  // {
  //   std::cerr << "[SocketCAN_CPP] Couldn't Read from can" << std::endl;
  // }
  // else
  // {
  //   std::cout << "ID: " << std::hex << std::uppercase << std::setw(8)
  //             << std::setfill('0') << frame.can_id;
  //   std::cout << " Data: " << std::hex << std::uppercase;
  //   for (size_t i = 0; i < 8; i++)
  //   {
  //     std::cout << (0xff & (unsigned int)frame.data[i]) << " ";
  //   }
  //   std::cout << std::endl;
  // }
  // struct can_frame frame;
  // const auto nbytes = read(m_iDevHandler, &frame, sizeof(struct can_frame));
  // LOG_INFO() << "::Receive: read " << nbytes << " " << frame.can_id << '\n';

  if (frame_num > MAX_CAN_RECV_FRAME_LEN || frame_num < 0)
  {
    LOG_ERROR() << "recv can frame num not in range[0, " << MAX_CAN_RECV_FRAME_LEN
                << "], frame_num:" << frame_num;
    // TODO(Authors): check the difference of returning frame_num/error_code
    return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
  }
  if (m_fdMode == MODE_CANFD_MTU)
  {

    for (int32_t i = 0; i < frame_num && i < MAX_CAN_RECV_FRAME_LEN; ++i)
    {
      CanFrame cf;
      memset(m_fdRecvFrames[i].data, 0, CANBUS_MESSAGE_LENGTH_FD); // dfeault is 0
      auto ret = read(m_iDevHandler, &m_fdRecvFrames[i], sizeof(m_fdRecvFrames[i]));
      // LOG_INFO() << "SocketCanClient::Receive: " << ret << " " << i << '\n';
      if (ret < 0)
      {
        LOG_ERROR() << "receive message failed, error code: " << ret;
        return ErrorCode::CAN_CLIENT_ERROR_BASE;
      }
      if (m_fdRecvFrames[i].len > static_cast<uint8_t>(CANBUS_MESSAGE_LENGTH_FD) ||
          m_fdRecvFrames[i].len == 0)
      {
        LOG_ERROR() << "recv_frames_[" << i
                    << "].can_dlc = " << recv_frames_[i].can_dlc
                    << ", which is not equal to can message data length ("
                    << CANBUS_MESSAGE_LENGTH << ").";
        return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
      }
      cf.id = m_fdRecvFrames[i].can_id;
      cf.len = m_fdRecvFrames[i].len;
      std::memcpy(cf.data, m_fdRecvFrames[i].data, m_fdRecvFrames[i].len);
      // get bus timestamp
      ioctl(m_iDevHandler, SIOCGSTAMP, &cf.timestamp);
      // LOG_INFO() << i << cf.CanFrameString();
      frames->push_back(cf);
    }
  }
  else
  {

    for (int32_t i = 0; i < frame_num && i < MAX_CAN_RECV_FRAME_LEN; ++i)
    {
      CanFrame cf;
      auto ret = read(m_iDevHandler, &recv_frames_[i], sizeof(recv_frames_[i]));
      // LOG_INFO() << "SocketCanClient::Receive: " << ret << " " << i << '\n';
      if (ret < 0)
      {
        LOG_ERROR() << "receive message failed, error code: " << ret;
        return ErrorCode::CAN_CLIENT_ERROR_BASE;
      }
      if (recv_frames_[i].can_dlc > static_cast<uint8_t>(CANBUS_MESSAGE_LENGTH) ||
          recv_frames_[i].can_dlc == 0)
      {
        LOG_ERROR() << "recv_frames_[" << i
                    << "].can_dlc = " << recv_frames_[i].can_dlc
                    << ", which is not equal to can message data length ("
                    << CANBUS_MESSAGE_LENGTH << ").";
        return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
      }
      cf.id = recv_frames_[i].can_id;
      cf.len = recv_frames_[i].can_dlc;
      std::memcpy(cf.data, recv_frames_[i].data, recv_frames_[i].can_dlc);
      // get bus timestamp
      ioctl(m_iDevHandler, SIOCGSTAMP, &cf.timestamp);
      // LOG_INFO() << i << cf.CanFrameString();
      frames->push_back(cf);
    }
  }

  return ErrorCode::OK;
}

NS_ZF::ErrorCode SocketCanClient::SetFilters(const std::string &str)
{
  m_filter->SetFilters(m_iDevHandler, str);
  return ErrorCode::OK;
}

NS_ZF::ErrorCode SocketCanClient::Wait(const std::chrono::nanoseconds timeout)
{
  // returns a value of the same type as timeout with a value of zero: as timeout > 0
  if (decltype(timeout)::zero() < timeout)
  {
    auto c_timeout = to_timeval(timeout);
    auto dSet = SingleSet(m_iDevHandler);
    LOG_INFO() << "Begin to Wait";
    // Wait
    if (0 == select(m_iDevHandler + 1, NULL, &dSet, NULL, &c_timeout))
    {
      LOG_ERROR() << "select Wait Timeout";

      // throw SocketCanTimeout{"CAN Send Timeout"};
      return ErrorCode::CAN_CLIENT_ERROR_TIMEOUT;
    }
    // lint --e{9130, 9123, 9125, 1924, 9126} NOLINT
    if (!FD_ISSET(m_iDevHandler, &dSet))
    {
      LOG_ERROR() << "FD_ISSET Wait Timeout";
      return ErrorCode::CAN_CLIENT_ERROR_TIMEOUT;
    }
  }
  LOG_INFO() << "EXIT to Wait";
  return ErrorCode::OK;
}

std::string SocketCanClient::GetErrorString(const int32_t /*status*/)
{
  return "";
}

fd_set SocketCanClient::SingleSet(int32_t file_descriptor) noexcept
{
  fd_set descriptor_set;
  // TODO(c.ho) sort through all these MISRA errors...
  // lint -save -e9146 NOLINT
  // lint --e{9063, 9036, 9084, 9027, 9033, 550, 717, 9001, 9093, 953} NOLINT
  FD_ZERO(&descriptor_set);
  // lint --e{9063, 9036, 9084, 9027, 9033, 550, 9123, 9125, 9126, 1924, 9130} NOLINT
  FD_SET(file_descriptor, &descriptor_set);
  // lint -restore NOLINT
  return descriptor_set;
}
END_NS_ZF_DRIVER_CANBUS
