#include "zf_hal_can_driver/client/pcan/pcan_client.h"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/sockios.h>
#include <sys/time.h>

/**
 * @namespace zf::driver::canbus
 */
BEGIN_NS_ZF_DRIVER_CANBUS

#define CAN_ID_MASK 0x1FFFF800U // can_filter mask

bool PCanClient::Init(const CANCardParameter &parameter)
{
  m_canParam = parameter;
  if (static_cast<uint8_t>(m_canParam.channelID) > (m_canParam.portsNum) || m_canParam.channelID < 0)
  {
    LOG_ERROR() << "Can port number [" << m_canParam.channelID << "] is out of range [0, "
                << m_canParam.portsNum << ") !";
    return false;
  }
  InitBaud();
  m_fdMode = m_canParam.enableFD ? true : false;
  return true;
}

PCanClient::PCanClient()
{
  // m_filter = std::make_unique<CanFilterList>();
}

PCanClient::~PCanClient()
{
  if (m_iPcanHandler)
  {
    Close();
  }
}

ErrorCode PCanClient::Open()
{
  if (m_bIsOpened)
  {
    return ErrorCode::OK;
  }
  if (!CheckForLibrary())
  {
    LOG_ERROR() << "CheckForLibrary failed [brand, channel]: [" << int(m_canParam.brand) << ", " << int(m_canParam.channelID) << "]";
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }
  if (!LookUpChannels())
  {
    LOG_ERROR() << "no device found with [brand, channel]: [" << int(m_canParam.brand) << ", " << int(m_canParam.channelID) << "]";
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }
  TPCANStatus stsResult;
  // Initialization of the selected channel
  LOG_INFO() << "m_fdMode " << m_fdMode << ", m_iPcanHandler " << m_iPcanHandler << ", m_iBaudrate " << m_iBaudrate;
  if (m_fdMode)
    stsResult = CAN_InitializeFD(m_iPcanHandler, m_sBitrateFD);
  else
    stsResult = CAN_Initialize(m_iPcanHandler, m_iBaudrate);
  if (stsResult != PCAN_ERROR_OK)
  {
    LOG_ERROR() << "Can not initialize. Please check the defines in the code [brand, channel]: [" << int(m_canParam.brand) << ", " << int(m_canParam.channelID) << "]";
    ShowStatus(stsResult);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  LOG_INFO() << "CAN Initialized ";
  m_bIsOpened = true;
  return ErrorCode::OK;
}

void PCanClient::Close()
{
  if (m_bIsOpened)
  {
    m_bIsOpened = false;
    CAN_Uninitialize(PCAN_NONEBUS);
  }
}

// Synchronous transmission of CAN messages
ErrorCode PCanClient::Send(const std::vector<CanFrame> &frames,
                           const int32_t &frame_num,
                           const std::chrono::nanoseconds &)
{
  CHECK_EQ(frames.size(), static_cast<size_t>(frame_num));

  if (!m_bIsOpened)
  {
    LOG_ERROR() << "PCanClient::Send: Nvidia socket can client has not been initiated! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
  }

  // if (Wait(timeout) != ErrorCode::OK)
  // {
  //   LOG_ERROR() << "PCanClient::Send: Timeout!";
  //   return ErrorCode::CAN_CLIENT_ERROR_TIMEOUT;
  // }

  for (size_t i = 0; i < frames.size() && i < MAX_CAN_SEND_FRAME_LEN; ++i)
  {
    if (frames[i].len > static_cast<uint8_t>(CANBUS_MESSAGE_LENGTH) || frames[i].len == 0)
    {
      LOG_ERROR() << "frames[" << i << "].len = " << frames[i].len
                  << ", which is not equal to can message data length ("
                  << CANBUS_MESSAGE_LENGTH << ").";
      return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
    }
    TPCANStatus stsResult;

    if (m_fdMode)
    {
      // Sends a CAN-FD message with standard ID, 64 data bytes, and bitrate switch
      TPCANMsgFD msgCanMessageFD;
      msgCanMessageFD.ID = frames[i].id;
      msgCanMessageFD.DLC = frames[i].len;
      msgCanMessageFD.MSGTYPE = PCAN_MESSAGE_FD | PCAN_MESSAGE_BRS;
      std::memcpy(msgCanMessageFD.DATA, frames[i].data, frames[i].len);

      stsResult = CAN_WriteFD(m_iPcanHandler, &msgCanMessageFD);
    }
    else
    {
      TPCANMsg msgCanMessage;
      msgCanMessage.ID = frames[i].id;
      msgCanMessage.LEN = frames[i].len;
      msgCanMessage.MSGTYPE = PCAN_MESSAGE_EXTENDED;
      std::memcpy(msgCanMessage.DATA, frames[i].data, frames[i].len);
      stsResult = CAN_Write(m_iPcanHandler, &msgCanMessage);
    }

    if (stsResult != PCAN_ERROR_OK)
    {
      LOG_ERROR() << "send message failed ";
      ShowStatus(stsResult);
      return ErrorCode::CAN_CLIENT_ERROR_BASE;
    }
  }
  return ErrorCode::OK;
}

// buf size must be 8 bytes, every time, we receive only one frame
ErrorCode PCanClient::Receive(std::vector<CanFrame> *const frames,
                              const int32_t &frame_num,
                              const std::chrono::nanoseconds &)
{
  if (!m_bIsOpened)
  {
    LOG_ERROR() << "PCanClient::Receive: can client is not init! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
  }

  if (frame_num > MAX_CAN_RECV_FRAME_LEN || frame_num < 0)
  {
    LOG_ERROR() << "recv can frame num not in range[0, " << MAX_CAN_RECV_FRAME_LEN
                << "], frame_num:" << frame_num;
    // TODO(Authors): check the difference of returning frame_num/error_code
    return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
  }

  for (int32_t i = 0; i < MAX_CAN_RECV_FRAME_LEN * 15; ++i)
  {
    CanFrame cf;
    TPCANStatus stsResult;
    if (m_fdMode)
    {
      TPCANMsgFD CANMsg;
      TPCANTimestampFD CANTimeStamp;
      stsResult = CAN_ReadFD(m_iPcanHandler, &CANMsg, &CANTimeStamp);
      if (stsResult != PCAN_ERROR_QRCVEMPTY)
      {
        cf.id = CANMsg.ID;
        cf.len = CANMsg.DLC;
        std::memcpy(cf.data, CANMsg.DATA, CANMsg.DLC);

        // get bus timestamp
        cf.timestamp.tv_sec = CANTimeStamp / 1000000;
        cf.timestamp.tv_usec = CANTimeStamp % 1000000;
        LOG_INFO() << i << cf.CanFrameString();
        frames->push_back(cf);
      }
    }
    else
    {
      TPCANMsg CANMsg;
      TPCANTimestamp itsTimeStamp;
      // struct timeval currentTime;
      // gettimeofday(&cf.timestamp, nullptr);
      stsResult = CAN_Read(m_iPcanHandler, &CANMsg, &itsTimeStamp);
      // LOG_DEBUG() << "BEGIN recv can frame , " << stsResult;
      if (stsResult != PCAN_ERROR_QRCVEMPTY)
      {
        cf.id = CANMsg.ID;
        cf.len = CANMsg.LEN;
        std::memcpy(cf.data, CANMsg.DATA, CANMsg.LEN);
        // get bus timestamp
        UINT64 microsTimestamp = ((UINT64)itsTimeStamp.micros + 1000 * (UINT64)itsTimeStamp.millis + 0x100000000 * 1000 * itsTimeStamp.millis_overflow);
        cf.timestamp.tv_sec = microsTimestamp / 1000000;
        cf.timestamp.tv_usec = microsTimestamp % 1000000;
        // LOG_INFO() << i << cf.CanFrameString();
        frames->push_back(cf);
      }
    }

    if (stsResult != PCAN_ERROR_OK && stsResult != PCAN_ERROR_QRCVEMPTY)
    {
      LOG_ERROR() << "receive message failed ";
      ShowStatus(stsResult);
      return ErrorCode::CAN_CLIENT_ERROR_BASE;
    }
    if (frame_num)
    {
      if (i >= frame_num)
      {
        break;
      }
    }
  }
  return ErrorCode::OK;
}

bool PCanClient::LookUpChannels()
{
  m_vecHandlers.clear();
  DWORD channelsCount;
  if (CAN_GetValue(PCAN_NONEBUS, PCAN_ATTACHED_CHANNELS_COUNT, &channelsCount, 4) == PCAN_ERROR_OK)
  {
    printf("Total of %d channels were found:\n", channelsCount);
    if (channelsCount > 0)
    {
      TPCANChannelInformation *channels = new TPCANChannelInformation[channelsCount];
      if (CAN_GetValue(PCAN_NONEBUS, PCAN_ATTACHED_CHANNELS, channels, channelsCount * sizeof(TPCANChannelInformation)) == PCAN_ERROR_OK)
      {
        for (int i = 0; i < (int)channelsCount; i++)
        {
          m_vecHandlers.push_back(channels[i].channel_handle);

          // printf("%d) ---------------------------\n", i + 1);
          // printf(" Name: %s\n", channels[i].device_name);
          // printf(" ID: %d\n", channels[i].device_id);
          // printf(" device_features: %d\n", channels[i].device_features);
          // printf(" Handle: 0x%X\n", channels[i].channel_handle);
          // printf("Controller: %d\n", channels[i].controller_number);
          // printf(" Condition: %d\n", channels[i].channel_condition);
          // printf(" . . . . . \n");
        }
      }
      delete[] channels;
      m_iPcanHandler = m_vecHandlers.at(m_canParam.channelID);
      //  m_iPcanHandler = PCAN_USBBUS1;
      return true;
    }
  }
  return false;
}

bool PCanClient::CheckForLibrary()
{
  // Check for dll file
  try
  {
    CAN_Uninitialize(PCAN_NONEBUS);
    return true;
  }
  catch (const std::exception &)
  {
    LOG_ERROR() << "Unable to find the library: PCANBasic::dll !\n";
  }
  return false;
}

void PCanClient::InitBaud()
{
  m_iBaudrate = PCAN_BAUD_500K;
  if (m_fdMode)
  {
    m_sBitrateFD = const_cast<LPSTR>("f_clock_mhz=20, nom_brp=5, nom_tseg1=2, nom_tseg2=1, nom_sjw=1, data_brp=2, data_tseg1=3, data_tseg2=1, data_sjw=1");
  }
  switch (m_canParam.baudrate)
  {
  case CANCardParameter::CANBaudrate::BCAN_BAUDRATE_100K:
    m_iBaudrate = PCAN_BAUD_100K;
    break;
  case CANCardParameter::CANBaudrate::BCAN_BAUDRATE_250K:
    m_iBaudrate = PCAN_BAUD_250K;
    break;
  case CANCardParameter::CANBaudrate::BCAN_BAUDRATE_500K:
    m_iBaudrate = PCAN_BAUD_500K;
    break;
  case CANCardParameter::CANBaudrate::BCAN_BAUDRATE_1M:
    m_iBaudrate = PCAN_BAUD_1M;
    break;
  default:
    break;
  }
}

void PCanClient::ShowStatus(TPCANStatus status)
{
  char buffer[4096];
  if (CAN_GetErrorText(status, 0x09, buffer) != PCAN_ERROR_OK)
    snprintf(buffer, 4096, "An error occurred. Error-code's text (%Xh) couldn't be retrieved", status);
  LOG_ERROR() << buffer << "\n";
}

NS_ZF::ErrorCode PCanClient::SetFilters(const std::string &str)
{
  // m_filter->SetFilters(m_iPcanHandler, str);
  // TPCANStatus result;
  // char strMsg[256];
  // DWORD iBuffer;
  // // The message filter is closed first to ensure the reception of the new range of IDs.
  // iBuffer = PCAN_FILTER_CLOSE;
  // result = CAN_SetValue(PCAN_USBBUS1, PCAN_MESSAGE_FILTER, &iBuffer, sizeof(iBuffer));
  // if (result != PCAN_ERROR_OK)
  // {
  //   // An error occurred, get a text describing the error and show it
  //   ShowStatus(result);
  //   return ErrorCode::CANBUS_ERROR;
  // }
  // else
  // {
  //   // The message filter is configured to receive the IDs 2,3,4 and 5 on the PCAN-USB, Channel 1
  //   result = CAN_FilterMessages(PCAN_USBBUS1, 2, 5, PCAN_MESSAGE_STANDARD);
  //   if (result != PCAN_ERROR_OK)
  //   {
  //     // An error occurred, get a text describing the error and show it
  //     ShowStatus(result);
  //     return ErrorCode::CANBUS_ERROR;
  //   }
  // }
  return ErrorCode::OK;
}

NS_ZF::ErrorCode PCanClient::Wait(const std::chrono::nanoseconds timeout)
{
  // returns a value of the same type as timeout with a value of zero: as timeout > 0
  // if (decltype(timeout)::zero() < timeout)
  // {
  //   auto c_timeout = to_timeval(timeout);
  //   auto dSet = SingleSet(m_iPcanHandler);
  //   LOG_INFO() << "Begin to Wait";
  //   // Wait
  //   if (0 == select(m_iPcanHandler + 1, NULL, &dSet, NULL, &c_timeout))
  //   {
  //     LOG_ERROR() << "select Wait Timeout";

  //     // throw SocketCanTimeout{"CAN Send Timeout"};
  //     return ErrorCode::CAN_CLIENT_ERROR_TIMEOUT;
  //   }
  //   // lint --e{9130, 9123, 9125, 1924, 9126} NOLINT
  //   if (!FD_ISSET(m_iPcanHandler, &dSet))
  //   {
  //     LOG_ERROR() << "FD_ISSET Wait Timeout";
  //     return ErrorCode::CAN_CLIENT_ERROR_TIMEOUT;
  //   }
  // }
  // LOG_INFO() << "EXIT to Wait";
  // return ErrorCode::OK;
}

std::string PCanClient::GetErrorString(const int32_t /*status*/)
{
  return "";
}

fd_set PCanClient::SingleSet(int32_t file_descriptor) noexcept
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
