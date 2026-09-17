// #include "zf_hal_can_driver/client/ethernet_can/ethernet_can_client.h"

// #include <string.h> //memset
// #include <netdb.h>  //htons
// #include <linux/if_ether.h>
// #include <linux/if_packet.h> //sockaddr_ll
// #include <linux/sockios.h>//SIOCGSTAMP
// #include <net/if.h>
// #include <sys/ioctl.h>

// #include "zf_hal_can_driver/client/socket_can/can_filter.h"
// // #include <linux/in.h>

// BEGIN_NS_ZF_DRIVER_CANBUS

// #define BUFFER_MAX 2048

// EthernetClient::EthernetClient(/* args */)
// {
//   m_filter = std::make_unique<CanFilterList>();
//   m_session = std::make_unique<NS_ZF_DRIVER_IO::Session>();
//   LOG_DEBUG() << "Create EthernetClient";
// }

// EthernetClient::~EthernetClient()
// {
// }

// bool EthernetClient::Init(const CANCardParameter &parameter)
// {
//   m_canParam = parameter;
//   if (m_canParam.deviceName.empty())
//   {
//     LOG_ERROR() << "Can deviceName empty!";
//     return false;
//   }
//   return true;
// }

// NS_ZF::ErrorCode EthernetClient::Open()
// {
//     LOG_INFO() << "Begin Open: " << m_session->fd();
//   auto t = m_session->Socket(PF_PACKET, SOCK_RAW, htons(ETH_P_IP));
//   if (t < 0)
//   {
//     LOG_ERROR() << "create EthernetClient socket failed";
//     return ErrorCode::CAN_CLIENT_ERROR_BASE;
//   }
//   LOG_INFO() <<t  << " Init: " << m_session->fd();

//   struct timeval tv = m_canParam.timestamp;
//   // tv.tv_sec = m_canParam.sec;                             /* 30 Secs Timeout */
//   // tv.tv_usec = m_canParam.timeout_ms * 1000; // Not init'ing this can cause strange errors???
//   m_session->SetSockopt(SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(struct timeval));

//   struct ifreq ifr;
//   memset(&ifr, 0x00, sizeof(ifr));
//   strncpy(ifr.ifr_name, m_canParam.deviceName.c_str(), sizeof(ifr.ifr_name));
//   // call ioctl system call to fetch iface index
//   if (ioctl(m_session->fd(), SIOCGIFHWADDR, &ifr) == -1)
//   {
//     LOG_ERROR() << "ioctl hwaddr error";
//     return ErrorCode::CAN_CLIENT_ERROR_BASE;
//   }
//   memcpy(m_frameMacSender.src_addr, ifr.ifr_hwaddr.sa_data, ETH_ALEN);
//   m_frameMacSender.type = htons(ETH_P_IP);
//   memset(&m_frameMacSender.dest_addr, 0xFF, ETH_ALEN);

//   if (ioctl(m_session->fd(), SIOCGIFINDEX, &ifr) == -1)
//   {
//     LOG_ERROR() << "ioctl index error";
//     return ErrorCode::CAN_CLIENT_ERROR_BASE;
//   }
//   LOG_INFO() << "if_index: " << ifr.ifr_ifindex;

//   struct sockaddr_ll sll;
//   memset(&sll, 0, sizeof(sll));
//   sll.sll_family = AF_PACKET;         // =PF_PACKET
//   sll.sll_protocol = htons(ETH_P_IP); //
//   sll.sll_ifindex = ifr.ifr_ifindex;
//   // sll.sll_pkttype = PACKET_BROADCAST;//a physical-layer broadcast packet
//   sll.sll_pkttype = PACKET_HOST; // a packet addressed to the local host
//   sll.sll_halen = ETH_ALEN;      // 硬件地址(MAC)的长度，ha是Hardware Address的意思
//   // Bind socket to server address
//   if (m_session->Bind((struct sockaddr *)&sll, sizeof(sll)) == -1)
//   {
//     LOG_ERROR() << "bind socket to network interface error !";
//     return ErrorCode::CAN_CLIENT_ERROR_BASE;
//   }
//   m_bIsOpened = true;
//   return ErrorCode::OK;
// }

// void EthernetClient::Close()
// {
//   if (m_bIsOpened)
//   {
//     m_bIsOpened = false;
//     m_session->Close();
//   }
// }

// NS_ZF::ErrorCode EthernetClient::Send(const std::vector<CanFrame> &frames, const int32_t &frame_num, const std::chrono::nanoseconds &timeout)
// {
//   CHECK_EQ(frames.size(), static_cast<size_t>(frame_num));
//   if (!m_bIsOpened)
//   {
//     LOG_ERROR() << "EthernetClient::Send: Nvidia Ethernet can client has not been initiated! Please init first!";
//     return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
//   }
//   const auto frame_size = sizeof(MAC_FRM_HDR) + sizeof(CAN_HDR);
//   char buffer[frame_size];

//   for (size_t i = 0; i < frames.size() && i < MAX_CAN_SEND_FRAME_LEN; ++i)
//   {
//     if (frames[i].len > static_cast<uint8_t>(CANBUS_MESSAGE_LENGTH) || frames[i].len == 0)
//     {
//       LOG_ERROR() << "frames[" << i << "].len = " << frames[i].len
//                   << ", which is not equal to can message data length ("
//                   << CANBUS_MESSAGE_LENGTH << ").";
//       return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
//     }
//     CAN_HDR header;
//     CanFrame2CAN_HDR(frames[i], header);
//     memset(&buffer, 0, frame_size);
//     // Copy the structures into the buffer
//     memcpy(buffer, &m_frameMacSender, sizeof(MAC_FRM_HDR));
//     memcpy(buffer + sizeof(MAC_FRM_HDR), &header, sizeof(CAN_HDR));
//     // Synchronous transmission of CAN messages
//     if (m_session->SendTo(&buffer, frame_size, 0, NULL, 0) == -1)
//     {
//       LOG_ERROR() << "send message failed.";
//       return ErrorCode::CAN_CLIENT_ERROR_BASE;
//     }
//   }
// }

// NS_ZF::ErrorCode EthernetClient::Receive(std::vector<CanFrame> *const frames, const int32_t &frame_num, const std::chrono::nanoseconds &timeout)
// {
//   if (!m_bIsOpened)
//   {
//     LOG_ERROR() << "SocketCanClient::Receive: Nvidia socket can client is not init! Please init first!";
//     return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
//   }
//   if (frame_num > MAX_CAN_RECV_FRAME_LEN || frame_num < 0)
//   {
//     LOG_ERROR() << "recv can frame num not in range[0, " << MAX_CAN_RECV_FRAME_LEN
//                 << "], frame_num:" << frame_num;
//     return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
//   }

//   char buf[BUFFER_MAX];
//   for (int32_t i = 0; i < frame_num && i < MAX_CAN_RECV_FRAME_LEN; ++i)
//   {
//     memset(&buf, 0, BUFFER_MAX);
//     int n_rd = m_session->RecvFrom(buf, BUFFER_MAX, 0, NULL, NULL);

//     if (n_rd != 88)
//     {
//       if (n_rd < 46)
//       {
//         LOG_ERROR() << "Incomplete packet: " << strerror(errno);
//         // continue;
//         return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
//       }
//       continue;
//     }
//     // LOG_INFO() << "n_rd is: " << n_rd;
//     CanFrame cf;
//     ParserBuffer(buf, cf);
//     // get bus timestamp
//     ioctl(m_session->fd(), SIOCGSTAMP, &cf.timestamp);

//     // LOG_INFO() << i << cf.CanFrameString();
//     frames->push_back(cf);
//   }
//   return ErrorCode::OK;
// }

// NS_ZF::ErrorCode EthernetClient::SetFilters(const std::string &str)
// {
//   // m_filter->SetFilters(m_session->fd(), str);
//   return ErrorCode::OK;
// }

// NS_ZF::ErrorCode EthernetClient::Wait(const std::chrono::nanoseconds timeout)
// {
//   // returns a value of the same type as timeout with a value of zero: as timeout > 0
//   if (decltype(timeout)::zero() < timeout)
//   {
//     return ErrorCode::CAN_CLIENT_ERROR_TIMEOUT;
//   }
//   LOG_INFO() << "EXIT to Wait";
//   return ErrorCode::OK;
// }

// std::string EthernetClient::GetErrorString(const int32_t status)
// {
//   return "";
// }

// void EthernetClient::CanFrame2CAN_HDR(const CanFrame &frame, CAN_HDR &header)
// {
//   header.ip_ver = (m_frameMacSender.src_addr[0] >> 1) & 0XF;
//   header.ip_len = m_frameMacSender.src_addr[0] & 0XF;
//   header.ip_tos = m_frameMacSender.src_addr[1];
//   header.canId = frame.channelID;
//   header.config_data_flag = 0;
//   header.canfd_flag = 0x01;
//   header.msg_id = frame.IdentifierID();
//   header.length = frame.len;
//   // for (size_t i = 0; i < frame.len && i < CANBUS_MESSAGE_LENGTH_FD; ++i) // max fd len is 64
//   // {
//   //   header.data[i] = frame.data[i];
//   // }
//   auto size = (frame.len > CANBUS_MESSAGE_LENGTH_FD) ? CANBUS_MESSAGE_LENGTH_FD : frame.len;
//   std::memcpy(header.data, frame.data, size);
// }

// bool EthernetClient::ParserBuffer(char *buf, CanFrame &frame)
// {
//   MAC_FRM_HDR *mac_hdr; // define a Ethernet frame header
//   int AND_LOGIC = 0xFF;

//   mac_hdr = reinterpret_cast<MAC_FRM_HDR *>(buf); // buf is what we got from the socket program
//   m_frameMacSender.type = mac_hdr->type;
//   memcpy(m_frameMacSender.dest_addr, mac_hdr->src_addr, ETH_ALEN);

//   CAN_HDR *can_data = reinterpret_cast<CAN_HDR *>(buf + sizeof(MAC_FRM_HDR));
//   CAN_HDR2CanFrame(can_data, frame);

//   // ip_hdr = reinterpret_cast<IP_HDR *>(buf + sizeof(MAC_FRM_HDR));
//   // udp_hdr = buf + sizeof(MAC_FRM_HDR) + sizeof(IP_HDR); //if we want to analyses the UDP/TCP
//   // char *tmp1, *tmp2;
//   // tmp1 = mac_hdr->src_addr;
//   // tmp2 = mac_hdr->dest_addr;
//   // // /* print the MAC addresses of source and receiving host */
//   // printf("MAC: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X==>"
//   //        "%.2X:%.2X:%.2X:%.2X:%.2X:%.2X",
//   //        tmp1[0] & AND_LOGIC, tmp1[1] & AND_LOGIC, tmp1[2] & AND_LOGIC, tmp1[3] & AND_LOGIC,
//   //        tmp1[4] & AND_LOGIC, tmp1[5] & AND_LOGIC,
//   //        tmp2[0] & AND_LOGIC, tmp2[1] & AND_LOGIC, tmp2[2] & AND_LOGIC, tmp2[3] & AND_LOGIC,
//   //        tmp2[4] & AND_LOGIC, tmp2[5] & AND_LOGIC);
//   // std::cout << "mac_hdr->type: " << ntohs(mac_hdr->type) << std::endl;
//   // tmp1 = (char *)&ip_hdr->ip_src;
//   // tmp2 = (char *)&ip_hdr->ip_dest;
//   // /* print the IP addresses of source and receiving host */
//   // printf("IP: %d.%d.%d.%d => %d.%d.%d.%d",
//   //        tmp1[0] & AND_LOGIC, tmp1[1] & AND_LOGIC, tmp1[2] & AND_LOGIC, tmp1[3] & AND_LOGIC,
//   //        tmp2[0] & AND_LOGIC, tmp2[1] & AND_LOGIC, tmp2[2] & AND_LOGIC, tmp2[3] & AND_LOGIC);
//   // /* print the IP protocol which was used by the socket communication */
//   // switch (ip_hdr->ip_protocol)
//   // {
//   // case IPPROTO_ICMP:
//   //   LOGI("ICMP\n");
//   //   break;
//   // case IPPROTO_IGMP:
//   //   LOGI("IGMP\n");
//   //   break;
//   // case IPPROTO_IPIP:
//   //   LOGI("IPIP\n");
//   //   break;
//   // case IPPROTO_TCP:
//   // case IPPROTO_UDP:
//   //   LOGI("Protocol: %s\n", ip_hdr->ip_protocol == IPPROTO_TCP ? "TCP" : "UDP");
//   //   // LOGI("Source port: %u, destination port: %u", udp_hdr->s_port, udp_hdr->d_port);
//   //   break;
//   // case IPPROTO_RAW:
//   //   LOGI("RAW\n");
//   //   break;
//   // default:
//   //   printf("Unknown, please query in inclued/linux/in.h\n");
//   //   break;
//   // }
//   return true;
// }

// void EthernetClient::CAN_HDR2CanFrame(const CAN_HDR *can_data, CanFrame &frame)
// {
//   frame.id = ntohl(can_data->msg_id);
//   frame.len = can_data->length;
//   std::memcpy(frame.data, can_data->data, can_data->length);
// }

// END_NS_ZF_DRIVER_CANBUS

// // using namespace zf::driver::io;

// // int main(int argc, char *argv[])
// // {
// //   EthernetClient serv;
// //   serv.Init();
// //   return 0;
// // }