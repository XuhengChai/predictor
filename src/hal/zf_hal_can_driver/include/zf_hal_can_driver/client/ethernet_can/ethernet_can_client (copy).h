// #ifndef ZF_HAL_CAN_CLIENT_ETHERNET_H
// #define ZF_HAL_CAN_CLIENT_ETHERNET_H

// #include <sstream>
// #include <iomanip>

// #include "zf_io/zf_io_session.h"
// #include "zf_hal_can_driver/client/can_client.h"

// /**
//  * @namespace zf::driver::canbus
//  */
// BEGIN_NS_ZF_DRIVER_CANBUS

// // 结构体定义中的 __attribute__((__packed__)) ，它告诉编译器：字段内存无须对齐，避免造成空洞
// typedef struct mac_frm_hdr
// {
//   char dest_addr[6]; // destination MAC address shall be defined first.
//   char src_addr[6];
//   short type; // 2 bytes; type, in network byte order
// } __attribute__((packed)) MAC_FRM_HDR;

// typedef struct ip_hdr
// { // header of IPV4
// #ifdef __LITTLE_ENDIAN_BIFIELD
//   u_char ip_len : 4, ip_ver : 4;
// #else
//   u_char ip_ver : 4, ip_len : 4;
// #endif

//   u_char ip_tos;
//   u_short ip_total_len;
//   u_short ip_id;
//   u_short ip_flags;
//   u_char ip_ttl;
//   u_char ip_protocol;
//   u_short ip_chksum;
//   uint32_t ip_src;
//   uint32_t ip_dest;
// } __attribute__((packed)) IP_HDR;

// typedef struct can_hdr
// {
//   // header of IPV4
// #ifdef __LITTLE_ENDIAN_BIFIELD
//   u_char ip_len : 4, ip_ver : 4;
// #else
//   u_char ip_ver : 4, ip_len : 4;
// #endif
//   u_char ip_tos;
//   uint8_t canId;
//   uint8_t config_data_flag;
//   uint8_t canfd_flag;
//   uint32_t msg_id;
//   // uint8_t msg_id[4];
//   uint8_t length;
//   uint8_t data[CANBUS_MESSAGE_LENGTH_FD];
// } __attribute__((packed)) CAN_HDR;

// class EthernetClient : public CanClient
// {
// public:
//   EthernetClient(/* args */);
//   ~EthernetClient();
//   // void Init(const char *iface = "eth4");

//   /**
//    * @brief Initialize the CAN client by specified CAN card parameters.
//    * @param parameter CAN card parameters to initialize the CAN client.
//    * @return If the initialization is successful.
//    */
//   bool Init(const CANCardParameter &parameter) override;

//   /**
//    * @brief Open the CAN client.
//    * @return The status of the Open action which is defined by
//    *         NS_ZF::ErrorCode.
//    */
//   NS_ZF::ErrorCode Open() override;

//   /**
//    * @brief Close the CAN client.
//    */
//   void Close() override;

//   /**
//    * @brief Send messages
//    * @param frames The messages to send.
//    * @param frame_num The amount of messages to send.
//    * @return The status of the sending action which is defined by
//    *         NS_ZF::ErrorCode.
//    */
//   NS_ZF::ErrorCode Send(const std::vector<CanFrame> &frames,
//                         const int32_t &frame_num,
//                         const std::chrono::nanoseconds &timeout = std::chrono::nanoseconds::zero()) override;

//   /**
//    * @brief Receive messages
//    * @param frames The messages to receive.
//    * @param frame_num The amount of messages to receive.
//    * @return The status of the receiving action which is defined by
//    *         NS_ZF::ErrorCode.
//    */
//   NS_ZF::ErrorCode Receive(std::vector<CanFrame> *const frames,
//                            const int32_t &frame_num,
//                            const std::chrono::nanoseconds &timeout = std::chrono::nanoseconds::zero()) override;

//   NS_ZF::ErrorCode SetFilters(const std::string &str) override;

//   NS_ZF::ErrorCode Wait(const std::chrono::nanoseconds timeout) override;

//   /**
//    * @brief Get the error string.
//    * @param status The status to get the error string.
//    */
//   std::string GetErrorString(const int32_t status) override;

// private:
//   void CanFrame2CAN_HDR(const CanFrame &frame, CAN_HDR &header);
//   void CAN_HDR2CanFrame(const CAN_HDR *header, CanFrame &frame);
//   bool ParserBuffer(char *, CanFrame &frame);
//   MAC_FRM_HDR m_frameMacSender;
//   std::unique_ptr<NS_ZF_DRIVER_IO::Session> m_session;
// };

// END_NS_ZF_DRIVER_CANBUS

// #endif // ZF_HAL_CAN_CLIENT_ETHERNET_H