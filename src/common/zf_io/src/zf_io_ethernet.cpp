#include "zf_io/zf_io_ethernet.h"

#include <string.h> //memset
#include <netdb.h>  //htons
#include <linux/if_ether.h>
#include <linux/if_packet.h> //sockaddr_ll
#include <linux/sockios.h>   //SIOCGSTAMP

#include <net/if.h>
#include <sys/ioctl.h>
// #include "zf_framework_transport/shm/shm_public.h"
#include "zf_framework_transport/transport.h"
#include "zf_global/util/logger.h"

// #include <linux/in.h>
// NS_ZF::LOGGER::FileLogger LOG_FILE_IPM;

BEGIN_NS_ZF_DRIVER_IO

EthernetClient::EthernetClient(/* args */)
    : m_bIsShutdown(true)
{
  // m_segment = std::make_shared<NS_ZF_FRAMEWORK::PosixSegmentMutex>(SHARED_MEMORY_KEY_TC397);
  // m_segment->OpenOrCreate(BUFFER_CAN_DATA_397, BUFFER_MAX); // 创建共享锁以及内存映射
  InitTransmit();
  // LOG_FILE_IPM.SetFileName("build_at_" __DATE__ "_origin_ipm.log");
}

void EthernetClient::InitTransmit()
{
  NS_ZF_FRAMEWORK::RoleAttributes attr;
  attr.SetHostIp(NS_ZF::GlobalData::Instance().HostIp());
  attr.SetChannelName(SHARED_MEMORY_KEY_TC397);
  NS_ZF_FRAMEWORK::Identity id;
  attr.SetId(id.HashValue());
  m_transmitterEthCAN = NS_ZF_FRAMEWORK::Transport::Instance()
                            .CreateTransmitter<NS_ZF_FRAMEWORK::MsgBase>(
                                attr, NS_ZF_FRAMEWORK::OptionalMode::SHM);
}

bool EthernetClient::ParserBuffer(char *buf)
{
  MAC_FRM_HDR *mac_hdr;                           // define a Ethernet frame header
  mac_hdr = reinterpret_cast<MAC_FRM_HDR *>(buf); // buf is what we got from the socket program
  // mac_hdr->type = ntohs(mac_hdr->type); // not used, so do not need ntohs
  m_frameMacSender.type = mac_hdr->type;
  memcpy(m_frameMacSender.dest_addr, mac_hdr->src_addr, ETH_ALEN);

  auto can_data = reinterpret_cast<CAN_HDR *>(buf + sizeof(MAC_FRM_HDR));
  can_data->msg_id = ntohl(can_data->msg_id);

  std::string encodedBuf(buf, sizeof(MAC_FRM_HDR) + sizeof(CAN_HDR));
  // msg->ParseFromString(encodedImage);
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  long long nanoseconds =
      ts.tv_sec * 1000000000LL + ts.tv_nsec;
  auto send_msg = std::make_shared<NS_ZF_FRAMEWORK::MsgBase>(
      encodedBuf, nanoseconds);
  m_transmitterEthCAN->Transmit(send_msg);
  // LOG_FILE_IPM() << "[" << ts.tv_sec << "." << std::setw(9) << std::setfill('0') << ts.tv_nsec
  //                << "]-" << CanFrameString(can_data);
  return true;
  // std::vector<char> vec(buf, buf + sizeof(MAC_FRM_HDR) + sizeof(CAN_HDR));
  // // sizeof(buf) / sizeof(buf[0]) = 8, wrong
  // // std::vector<char> vec(buf, buf + BUFFER_CAN_DATA_397));
  // // printf("client pub %s\n", buffer);

  // if (m_segment->Notify(vec))
  // {
  //   // printf("client pub %s\n", buf);
  //   // std::cout << CanFrameString(can_data).c_str() << std::endl;
  //   struct timespec ts;
  //   clock_gettime(CLOCK_REALTIME, &ts);
  //   LOG_FILE_IPM() << "[" << ts.tv_sec << "." << std::setw(9) << std::setfill('0') << ts.tv_nsec
  //                  << "]-" << CanFrameString(can_data);
  //   // write_i++;
  //   return true;
  // }
  // return false;

  // ip_hdr = reinterpret_cast<IP_HDR *>(buf + sizeof(MAC_FRM_HDR));
  // udp_hdr = buf + sizeof(MAC_FRM_HDR) + sizeof(IP_HDR); //if we want to analyses the UDP/TCP
  // char *tmp1, *tmp2;
  // int AND_LOGIC = 0xFF;
  // tmp1 = mac_hdr->src_addr;
  // tmp2 = mac_hdr->dest_addr;
  // // /* print the MAC addresses of source and receiving host */
  // printf("MAC: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X==>"
  //        "%.2X:%.2X:%.2X:%.2X:%.2X:%.2X",
  //        tmp1[0] & AND_LOGIC, tmp1[1] & AND_LOGIC, tmp1[2] & AND_LOGIC, tmp1[3] & AND_LOGIC,
  //        tmp1[4] & AND_LOGIC, tmp1[5] & AND_LOGIC,
  //        tmp2[0] & AND_LOGIC, tmp2[1] & AND_LOGIC, tmp2[2] & AND_LOGIC, tmp2[3] & AND_LOGIC,
  //        tmp2[4] & AND_LOGIC, tmp2[5] & AND_LOGIC);
  // std::cout << "mac_hdr->type: " << mac_hdr->type << std::endl;
  // std::cout << CanFrameString(can_data).c_str() << std::endl;
}

EthernetClient::~EthernetClient()
{
  Shutdown();
}

ErrorCode EthernetClient::Open(const char *iface)
{

  if (this->Socket(PF_PACKET, SOCK_RAW, htons(ETH_P_IP)) < 0)
  {
    LOG_ERROR() << "create EthernetClient socket failed";
    return ErrorCode::SOCKET_ERROR;
  }
  LOG_INFO() << "Init: " << this->fd();

  // struct timeval tv = m_canParam.timestamp;
  // // tv.tv_sec = m_canParam.sec;                             /* 30 Secs Timeout */
  // // tv.tv_usec = m_canParam.timeout_ms * 1000; // Not init'ing this can cause strange errors???
  // this->SetSockopt(SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(struct timeval));

  struct ifreq ifr;
  memset(&ifr, 0x00, sizeof(ifr));
  strncpy(ifr.ifr_name, iface, sizeof(ifr.ifr_name));
  // call ioctl system call to fetch iface index
  if (ioctl(this->fd(), SIOCGIFHWADDR, &ifr) == -1)
  {
    LOG_ERROR() << "ioctl hwaddr error";
    return ErrorCode::SOCKET_ERROR;
  }
  memcpy(m_frameMacSender.src_addr, ifr.ifr_hwaddr.sa_data, ETH_ALEN);
  m_frameMacSender.type = htons(ETH_P_IP);
  memset(&m_frameMacSender.dest_addr, 0xFF, ETH_ALEN);

  if (ioctl(this->fd(), SIOCGIFINDEX, &ifr) == -1)
  {
    LOG_ERROR() << "ioctl index error";
    return ErrorCode::SOCKET_ERROR;
  }
  LOG_INFO() << "if_index: " << ifr.ifr_ifindex;

  struct sockaddr_ll sll;
  memset(&sll, 0, sizeof(sll));
  sll.sll_family = AF_PACKET;         // =PF_PACKET
  sll.sll_protocol = htons(ETH_P_IP); //
  sll.sll_ifindex = ifr.ifr_ifindex;
  // sll.sll_pkttype = PACKET_BROADCAST;//a physical-layer broadcast packet
  sll.sll_pkttype = PACKET_HOST; // a packet addressed to the local host
  sll.sll_halen = ETH_ALEN;      // 硬件地址(MAC)的长度，ha是Hardware Address的意思
  // Bind socket to server address
  if (this->Bind((struct sockaddr *)&sll, sizeof(sll)) == -1)
  {
    LOG_ERROR() << "bind socket to network interface error !";
    return ErrorCode::SOCKET_ERROR;
  }
  // Parameters
  // desired 	- 	value to assign
  // order 	- 	memory order constraints to enforce
  m_bIsShutdown.exchange(false); // Return The value of the atomic variable before the call
  return ErrorCode::OK;
}

void EthernetClient::LoopReceive()
{
  int n_rd;
  char buf[BUFFER_MAX];
  while (1)
  {
    if (m_bIsShutdown.load())
    {
      LOG_DEBUG() << "LoopReceive is shutdown.";
      break;
    }
    // LOG_DEBUG() << "LoopReceive begin.";
    n_rd = this->RecvFrom(buf, BUFFER_MAX, 0, NULL, NULL);
    // LOG_DEBUG() << "RecvFrom ." << n_rd;
    if (n_rd < 46)
    {
      LOG_ERROR() << "Incomplete packet: " << strerror(errno);
      continue;
      // this->Close();
      // break;
    }
    if (n_rd != BUFFER_CAN_DATA_397)
    {
      continue;
    }
    // LOG_INFO() << "n_rd is: " << n_rd;
    // timeval timestamp;
    // ioctl(this->fd(), SIOCGSTAMP, &timestamp);
    // LOG_INFO() << "[" << timestamp.tv_sec << "." << std::setw(6) << std::setfill('0') << timestamp.tv_usec
    //            << "] ";
    ParserBuffer(buf);
  }
  Shutdown();
}

bool EthernetClient::Send(CAN_HDR candata)
{
  u_char des[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
  memcpy(m_frameMacSender.dest_addr, des, ETH_ALEN);

  u_char *tmp1, *tmp2;
  int AND_LOGIC = 0xFF;
  tmp1 = m_frameMacSender.src_addr;
  tmp2 = m_frameMacSender.dest_addr;
  // /* print the MAC addresses of source and receiving host */
  printf("%d, MAC: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X==>"
         "%.2X:%.2X:%.2X:%.2X:%.2X:%.2X",
         ntohs(m_frameMacSender.type),
         tmp1[0] & AND_LOGIC, tmp1[1] & AND_LOGIC, tmp1[2] & AND_LOGIC, tmp1[3] & AND_LOGIC,
         tmp1[4] & AND_LOGIC, tmp1[5] & AND_LOGIC,
         tmp2[0] & AND_LOGIC, tmp2[1] & AND_LOGIC, tmp2[2] & AND_LOGIC, tmp2[3] & AND_LOGIC,
         tmp2[4] & AND_LOGIC, tmp2[5] & AND_LOGIC);
  std::cout << std::endl;
  std::cout << CanFrameString(&candata).c_str() << std::endl;
  std::cout << std::endl;

  const auto frame_size = sizeof(MAC_FRM_HDR) + sizeof(CAN_HDR);
  char buffer[frame_size];
  memset(&buffer, 0, frame_size);
  // Copy the structures into the buffer
  memcpy(buffer, &m_frameMacSender, sizeof(MAC_FRM_HDR));
  candata.msg_id = htonl(candata.msg_id);
  memcpy(buffer + sizeof(MAC_FRM_HDR), &candata, sizeof(CAN_HDR));
  // Synchronous transmission of CAN messages
  if (this->SendTo(&buffer, frame_size, 0, NULL, 0) == -1)
  {
    LOG_ERROR() << "send message failed. " << strerror(errno);
    return false;
  }
  return true;
}

void EthernetClient::Shutdown()
{
  if (m_bIsShutdown.exchange(true))
  {
    return;
  }
  this->Close();
}

std::string EthernetClient::CanFrameString(CAN_HDR *buf) const
{
  std::stringstream output_stream("");
  output_stream << "ID " << int(buf->canId)
                << ", " << int(buf->canfd_flag)
                << ", " << int(buf->length)
                << " msg id: " << buf->msg_id
                << " msg id: " << PGNFromCanId(buf->msg_id)
                << ", data:";
  // << ", len:" << static_cast<int>(len) << ", data:";
  // for (uint8_t i = 0; i < 4; ++i)
  // {
  //   output_stream << Int2Hex(buf->msg_id[i]) << " ";
  // }
  // output_stream<< ", data:";
  for (uint8_t i = 0; i < buf->length; ++i)
  {
    output_stream << Int2Hex(buf->data[i]) << " ";
  }
  output_stream << ",";
  return output_stream.str();
}

END_NS_ZF_DRIVER_IO
