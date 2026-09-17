
// #include <memory>
// // #include <mutex>
// #include <thread>
// // #include <unordered_map>
// // #include <vector>

// #include "zf_hal_can_driver/client/can_client.h"
// #include "zf_hal_can_driver/hal_can_driver_global.h"

// #include <sys/time.h>
// #include <chrono>
// #include <regex>
// #include "rclcpp/time.hpp"
// #include <unistd.h>
// #include <thread>
// #include "PCANBasic.h"

// #include "zf_hal_can_driver/hal_can_id.h"
// #include "zf_hal_can_driver/client/can_client_factory.h"
// #include "zf_global/zf_global_topic_name.h"
// #include "zf_global/util/logger.h"
// #include "zf_global/zf_global.h"

// using namespace NS_ZF::NS_DRIVER::NS_CANBUS;
// using namespace NS_ZF;

// TPCANHandle m_iPcanHandler = PCAN_USBBUS1;

// void Open()
// {

//     TPCANStatus stsResult;
//     // Initialization of the selected channel
//     // if (m_fdMode)
//     //   stsResult = CAN_InitializeFD(m_iPcanHandler, m_sBitrateFD);
//     // else
//     stsResult = CAN_Initialize(m_iPcanHandler, PCAN_BAUD_1M);
//     if (stsResult != PCAN_ERROR_OK)
//     {
//         std::cout << "Can not initialize. Please check the defines in the code [brand, channel]: [";
//     }
//     std::cout << "CAN_Initialized \n";
// }

// // buf size must be 8 bytes, every time, we receive only one frame
// void ReceiveNew()
// {
//     for (int32_t i = 0; i < 1; ++i)
//     {
//         TPCANStatus stsResult;
//         TPCANMsg CANMsg;
//         TPCANTimestamp CANTimeStamp;
//         stsResult = CAN_Read(m_iPcanHandler, &CANMsg, &CANTimeStamp);
//         std::cout << "BEGIN recv can frame , " << stsResult << std::endl;
//         if (stsResult != PCAN_ERROR_OK && stsResult != PCAN_ERROR_QRCVEMPTY)
//         {
//             std::cout << "receive message failed " << std::endl;
//         }
//     }
// }

// class Pcan
// {
// public:
//     // std::unique_ptr<CanClient> m_canClient;
//     // std::unique_ptr<std::thread> m_threadRecv;
//     std::thread *m_threadRecv;
//     int receive_error_count = {};
//     bool is_shutdown = true;
//     ~Pcan()
//     {
//         is_shutdown = false;
//         if (m_threadRecv->joinable())
//         {
//             m_threadRecv->join();
//         }
//     }

//     void Init()
//     {
//         CANCardParameter param;
//         param.brand = CANCardParameter::CANCardBrand::PCAN;
//         param.channelID = CANCardParameter::CANChannelId::CHANNEL_ID_ZERO;

//         if (!CheckForLibrary())
//             return;
//         param.deviceName = "pcan0";
//         param.enableFD = false;
//         param.baudrate = static_cast<CANCardParameter::CANBaudrate>(0);
//         // RCLCPP_INFO(this->get_logger(), "param.timestamp: %d.%d", param.timestamp.tv_sec, param.timestamp.tv_usec);
//         // CanClientFactory::Instance().RegisterCanClients();
//         // // m_canClient = std::move(CanClientFactory::Instance().CreateCANClient(param));
//         // m_canClient = CanClientFactory::Instance().CreateCANClient(param);
//         // if (m_canClient == nullptr)
//         // {
//         //     LOG_ERROR() << "Invalid can client.";
//         //     return;
//         // }

//         // auto ret = m_canClient->Open();
//         // if (ret != ErrorCode::OK)
//         // {
//         //     LOG_ERROR() << "Open can client failed";
//         //     return;
//         // }
//         Open();

//         // apply CAN filters
//         // auto filters = "0:0";
//         // m_canClient->SetFilters(filters);

//         // m_threadRecv = std::make_unique<std::thread>(&Pcan::Receive, this);
//         m_threadRecv = new std::thread(&Pcan::Receive, this);
//         // m_threadRecv = std::make_unique<std::thread>(&ReceiveNew);
//     }

//     void Receive()
//     {
//         while (is_shutdown)
//         {
//             for (int32_t i = 0; i < 1; ++i)
//             {
//                 TPCANStatus stsResult;
//                 TPCANMsg CANMsg;
//                 TPCANTimestamp CANTimeStamp;
//                 stsResult = CAN_Read(m_iPcanHandler, &CANMsg, &CANTimeStamp);
//                 // std::cout << "BEGIN recv can frame , " << stsResult << std::endl;
//                 if (stsResult != PCAN_ERROR_OK && stsResult != PCAN_ERROR_QRCVEMPTY)
//                 {
//                     std::cout << "receive message failed " << std::endl
//                               << stsResult;
//                     ShowStatus(stsResult);// error: read too late
//                 }
//             }
//             usleep(25000);
//         }
//     }

//     void ShowStatus(TPCANStatus status)
//     {
//         char buffer[4096];
//         if (CAN_GetErrorText(status, 0x09, buffer) != PCAN_ERROR_OK)
//             snprintf(buffer, 4096, "An error occurred. Error-code's text (%Xh) couldn't be retrieved", status);
//         LOG_ERROR() << buffer << "\n";
//     }

//     bool CheckForLibrary()
//     {
//         // Check for dll file
//         try
//         {
//             CAN_Uninitialize(PCAN_NONEBUS);
//             return true;
//         }
//         catch (const std::exception &)
//         {
//             std::cout << ("Unable to find the library: PCANBasic::dll !\n");
//         }

//         return false;
//     }
// };

int main(int argc, char *argv[])
{
    // Pcan t;
    // t.Init();
    // while (1)
    //     ;
    return 0;
}
