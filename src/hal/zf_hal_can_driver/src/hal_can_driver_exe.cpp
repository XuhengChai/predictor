#include <cstdio>
#include "zf_hal_can_driver/hal_can_driver_global.h"
#include "zf_hal_can_driver/hal_can_driver_exe.h"
#include "zf_global/common/zf_global_async_util.h"

#include <fcntl.h>
using namespace NS_ZF;
// #include <linux/can.h>
// #include <linux/sockios.h>

// int main(int argc, char ** argv)
// {
//   (void) argc;
//   (void) argv;

//   printf("hello world zf_hal_can_driver package\n");
//   return 0;
// }

#include <arpa/inet.h>
#include <stdio.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>

namespace SocketCAN_CPP
{
  SocketCAN::SocketCAN(const std::string &device_name)
  {
    int rc = 0;
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    // if (0 != fcntl(can_socket_, F_SETFL, O_NONBLOCK))
    // {
    //   throw std::runtime_error{"Failed to set CAN socket to nonblocking"};
    //   LOG_ERROR() << "Failed to set CAN socket to nonblocking[" << can_socket_ << "]: ";
    //   return;
    // }
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500 * 1000; // Not init'ing this can cause strange errors
    setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

    strcpy(ifr_.ifr_name, device_name.c_str());
    rc = ioctl(can_socket_, SIOCGIFINDEX, &ifr_);
    if (rc == -1)
    {
      std::cerr << "[SocketCAN_CPP] Error ioctl! wrong CAN device name"
                << std::endl;
      return;
    }

    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;

    rc = bind(can_socket_, (struct sockaddr *)&addr_, sizeof(addr_));
    if (rc == -1)
    {
      std::cerr << "[SocketCAN_CPP] Error Couldn't bind!" << std::endl;
      return;
    }
  }

  SocketCAN::~SocketCAN() { close(can_socket_); }

  struct can_frame SocketCAN::Read()
  {
    int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes < 0)
    {
      std::cerr << "[SocketCAN_CPP] Couldn't Read from can" << std::endl;
    }
    return frame;
  }

  int SocketCAN::Write(const struct can_frame &frame)
  {
    int n_bytes = write(can_socket_, &frame, sizeof(frame));
    return n_bytes;
  }
} // namespace SocketCAN_CPP

// int main(int argc, char const *argv[])
// {
//   SocketCAN_CPP::SocketCAN sCAN("can3");
//   while (1)
//   {
//     sCAN.Read();
//     std::cout << "ID: " << std::hex << std::uppercase << std::setw(8)
//               << std::setfill('0') << sCAN.frame.can_id;

//     std::cout << " Data: " << std::hex << std::uppercase;
//     for (size_t i = 0; i < 8; i++)
//     {
//       std::cout << (0xff & (unsigned int)sCAN.frame.data[i]) << " ";
//     }
//     std::cout << std::endl;
//     // sCAN.Write(sCAN.frame);
//     // sleep(1);
//     NS_ZF::USleep(100);
//   }

//   return 0;
// }
#include <sys/types.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <linux/in.h>

#define BUFFER_MAX 2048
typedef int int32;
typedef unsigned int u_int32;
typedef unsigned char u_char;
typedef unsigned short u_short;
#define LOGI printf

// 结构体定义中的 __attribute__((__packed__)) ，它告诉编译器：字段内存无须对齐，避免造成空洞
typedef struct mac_frm_hdr
{
  char dest_addr[6]; // destination MAC address shall be defined first.
  char src_addr[6];
  short type;
} __attribute__((packed)) MAC_FRM_HDR;

typedef struct ip_hdr
{ // header of IPV4
#ifdef __LITTLE_ENDIAN_BIFIELD
  u_char ip_len : 4, ip_ver : 4;
#else
  u_char ip_ver : 4, ip_len : 4;
#endif

  u_char ip_tos;
  u_short ip_total_len;
  u_short ip_id;
  u_short ip_flags;
  u_char ip_ttl;
  u_char ip_protocol;
  u_short ip_chksum;
  u_int32 ip_src;
  u_int32 ip_dest;
} __attribute__((packed)) IP_HDR;

typedef struct can_hdr
{
  u_char ip_len : 4;
  u_char ip_tos;
  u_char canId;
  u_char config_data_flag;
  u_char canfd_flag;
  u_int32 msg_id;
  u_char length;
  char data[6];
} __attribute__((packed)) CAN_HDR;

// 根据名字，获取网卡的序号
/**
 *  Fetch index of given iface.
 *  Arguments
 *      iface: name of given iface.
 *      s: socket for ioctl, optional.
 *  Returns
 *      Iface index(which is greater than 0) if success, -1 if error.
 **/
int fetch_iface_index(int s, const char *iface)
{
  // fill iface name to struct ifreq
  struct ifreq ifr;
  memset(&ifr, 0x00, sizeof(ifr));
  strncpy(ifr.ifr_name, iface, sizeof(ifr.ifr_name));
  // call ioctl system call to fetch iface index
  if (ioctl(s, SIOCGIFINDEX, &ifr) == -1)
  {
    return -1;
  }
  return ifr.ifr_ifindex;
}

int main(int argc, char *argv[])
{
  int SOCKET_SRC;
  char buf[BUFFER_MAX];
  int n_rd;

  if ((SOCKET_SRC = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_IP))) < 0)
  {
    fprintf(stderr, "create socket error.\n");
    exit(0);
  }

  // fill iface index to struct sockaddr_ll for binding
  int if_index = fetch_iface_index(SOCKET_SRC, "eth4");
  if (if_index == -1)
  {
    return -1;
  }
  std::cout << "if_index: " << if_index << std::endl;
  struct sockaddr_ll sll;
  bzero(&sll, sizeof(sll));
  sll.sll_family = AF_PACKET;
  sll.sll_ifindex = if_index;
  sll.sll_pkttype = PACKET_HOST;
  sll.sll_protocol = htons(ETH_P_IP);
  sll.sll_halen = ETH_ALEN; // 硬件地址(MAC)的长度，ha是Hardware Address的意思

  // call bind system call to bind socket with iface
  if (bind(SOCKET_SRC, (struct sockaddr *)&sll, sizeof(sll)) == -1)
  {
    fprintf(stderr, "bind socket error.\n");

    return -1;
  }

  while (1)
  {
    n_rd = recvfrom(SOCKET_SRC, buf, BUFFER_MAX, 0, NULL, NULL);
    std::cout << "n_rd: " << n_rd << std::endl;

    if (n_rd < 46)
    {
      perror("recvfrom():");
      printf("Incomplete packet (errno is %d)\n", errno);
      close(SOCKET_SRC);
      exit(0);
    }
    //  std::cout << "buf: " << buf << std::endl;
    MAC_FRM_HDR *mac_hdr; // define a Ethernet frame header
    IP_HDR *ip_hdr;       // define a IP header
    char *tmp1, *tmp2;
    int AND_LOGIC = 0xFF;

    mac_hdr = reinterpret_cast<MAC_FRM_HDR *>(buf); // buf is what we got from the socket program
    ip_hdr = reinterpret_cast<IP_HDR *>(buf + sizeof(MAC_FRM_HDR));
    // udp_hdr = buf + sizeof(MAC_FRM_HDR) + sizeof(IP_HDR); //if we want to analyses the UDP/TCP

    tmp1 = mac_hdr->src_addr;
    tmp2 = mac_hdr->dest_addr;
    /* print the MAC addresses of source and receiving host */
    printf("MAC: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X==>"
           "%.2X:%.2X:%.2X:%.2X:%.2X:%.2X",
           tmp1[0] & AND_LOGIC, tmp1[1] & AND_LOGIC, tmp1[2] & AND_LOGIC, tmp1[3] & AND_LOGIC,
           tmp1[4] & AND_LOGIC, tmp1[5] & AND_LOGIC,
           tmp2[0] & AND_LOGIC, tmp2[1] & AND_LOGIC, tmp2[2] & AND_LOGIC, tmp2[3] & AND_LOGIC,
           tmp2[4] & AND_LOGIC, tmp2[5] & AND_LOGIC);
    std::cout << "mac_hdr->type: " << mac_hdr->type << std::endl;

    tmp1 = (char *)&ip_hdr->ip_src;
    tmp2 = (char *)&ip_hdr->ip_dest;
    /* print the IP addresses of source and receiving host */
    printf("IP: %d.%d.%d.%d => %d.%d.%d.%d",
           tmp1[0] & AND_LOGIC, tmp1[1] & AND_LOGIC, tmp1[2] & AND_LOGIC, tmp1[3] & AND_LOGIC,
           tmp2[0] & AND_LOGIC, tmp2[1] & AND_LOGIC, tmp2[2] & AND_LOGIC, tmp2[3] & AND_LOGIC);
    /* print the IP protocol which was used by the socket communication */
    switch (ip_hdr->ip_protocol)
    {
    case IPPROTO_ICMP:
      LOGI("ICMP\n");
      break;
    case IPPROTO_IGMP:
      LOGI("IGMP\n");
      break;
    case IPPROTO_IPIP:
      LOGI("IPIP\n");
      break;
    case IPPROTO_TCP:
    case IPPROTO_UDP:
      LOGI("Protocol: %s\n", ip_hdr->ip_protocol == IPPROTO_TCP ? "TCP" : "UDP");
      // LOGI("Source port: %u, destination port: %u", udp_hdr->s_port, udp_hdr->d_port);
      break;
    case IPPROTO_RAW:
      LOGI("RAW\n");
      break;
    default:
      printf("Unknown, please query in inclued/linux/in.h\n");
      auto can_data = reinterpret_cast<CAN_HDR *>(buf + sizeof(MAC_FRM_HDR));
      tmp1 = (char *)&can_data->msg_id;
      tmp2 = (char *)&can_data->data;
      /* print the IP addresses of source and receiving host */
      printf("MSG_ID: %d.%d.%d.%d =>data %d.%d.%d.%d",
             tmp1[0] & AND_LOGIC, tmp1[1] & AND_LOGIC, tmp1[2] & AND_LOGIC, tmp1[3] & AND_LOGIC,
             tmp2[0] & AND_LOGIC, tmp2[1] & AND_LOGIC, tmp2[2] & AND_LOGIC, tmp2[3] & AND_LOGIC);
      std::cout << "CAN ID: " << int(can_data->canId) << std::endl;
      std::cout << "CAN length: " << int(can_data->length) << std::endl;

      break;
    }

    /* An Ethernet frame was written to buf, frame analysis can be processed here */
    /* Termination control */
  }
  close(SOCKET_SRC);
  return 0;
}

// int main(int argc, char const *argv[])
// {
//   int status, valread, client_fd;
//   char *hello = "Hello from client";
//   char buffer[1024] = {0};
//   // client_fd = socket(AF_INET, SOCK_STREAM, 0);
//   client_fd = socket(AF_INET, SOCK_DGRAM, 0);
//   if (client_fd < 0)
//   {
//     printf("\n Socket creation error \n");
//     return -1;
//   }
//   struct timeval tv;
//   struct hostent *server;
//   tv.tv_sec = 2;
//   tv.tv_usec = 0;
//   char *host = "192.168.55.110";

//   setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(struct timeval));

//   struct sockaddr_in serv_addr;
//   serv_addr.sin_family = AF_INET;
//   // serv_addr.sin_port = htons(44597);                   // Server port number
//   serv_addr.sin_addr.s_addr = inet_addr("192.168.55.110");
//   // Convert IPv4 and IPv6 addresses from text to binary
//   // form
//   // if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0)
//   // {
//   //   printf(
//   //       "\nInvalid address/ Address not supported \n");
//   //   return -1;
//   // }
//   status = connect(client_fd, (struct sockaddr *)&serv_addr,
//                    sizeof(serv_addr));
//   std::cout << "status: " << status << std::endl;
//   if (status < 0)
//   {
//     printf("\nConnection Failed \n");
//     return -1;
//   }
//   while (1)
//   {
//     printf("sleep 1 \n");

//     valread = read(client_fd, buffer, 1024);
//     printf("%s\n", buffer);
//     sleep(1);
//   }

//   send(client_fd, hello, strlen(hello), 0);
//   printf("Hello message sent\n");

//   // closing the connected socket
//   close(client_fd);
//   return 0;
// }

// int main(int argc, char const* argv[])
// {
//     int status, valread, client_fd;
//     struct sockaddr_in serv_addr;
//     char* hello = "Hello from client";
//     char buffer[1024] = { 0 };
//     if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
//         printf("\n Socket creation error \n");
//         return -1;
//     }

//     serv_addr.sin_family = AF_INET;
//     serv_addr.sin_port = htons(PORT);

//     // Convert IPv4 and IPv6 addresses from text to binary
//     // form
//     if (inet_pton(AF_INET, "192.168.55.110", &serv_addr.sin_addr)
//         <= 0) {
//         printf(
//             "\nInvalid address/ Address not supported \n");
//         return -1;
//     }

//     if ((status
//          = connect(client_fd, (struct sockaddr*)&serv_addr,
//                    sizeof(serv_addr)))
//         < 0) {
//         printf("\nConnection Failed \n");
//         return -1;
//     }
//     send(client_fd, hello, strlen(hello), 0);
//     printf("Hello message sent\n");
//     valread = read(client_fd, buffer, 1024);
//     printf("%s\n", buffer);

//     // closing the connected socket
//     close(client_fd);
//     return 0;
// }