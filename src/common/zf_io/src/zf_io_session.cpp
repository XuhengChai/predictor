#include "zf_io/zf_io_session.h"

#include <net/if.h>
#include <string.h>
#include <sys/ioctl.h>

// #include <sys/types.h>
// #include <linux/if_ether.h>
// #include <linux/if_packet.h>
// #include <linux/in.h>
// #include <arpa/inet.h>
// #include <stdio.h>
// #include <sys/socket.h>
// #include <netdb.h>
// #include <string.h>
// #include <unistd.h>
// #include <fcntl.h>

BEGIN_NS_ZF_DRIVER_IO

Session::Session() : Session(-1) {}

Session::Session(int fd) : fd_(fd)
{
}

/**
 * @brief 建立一个没有绑定地址的socket，返回socket文件描述符
 * @param domain 协议族；IPv4为AF_INET，IPv6为AF_INET6
 * @param type socket的传输方式；TCP为SOCK_STREAM，UDP为SOCK_DGRAM
 * @param protocol 指定socket使用的协议；通常情况下，一个协议族只支持一种协议，所以通常将protocol置为0，让系统选择匹配的协议；
 * @return socket文件描述符
 */
int Session::Socket(int domain, int type, int protocol)
{
  if (fd_ != -1)
  {
    LOG_INFO() << "session has hold a valid fd[" << fd_ << "]";
    return -1;
  }
  // int sock_fd = socket(domain, type | SOCK_NONBLOCK, protocol);
  int sock_fd = socket(domain, type, protocol);
  if (sock_fd != -1)
  {
    set_fd(sock_fd);
  }
  return sock_fd;
}

int Session::SetSockopt(int level, int optname, const void *optval, socklen_t optlen)
{
  if (fd_ == -1)
    return -1;
  return setsockopt(fd_, level, optname, optval, optlen);
}

/**
 * @brief 为未绑定地址的socket分配地址
 * @param addr 绑定的地址
 * @param addrlen addr结构的大小
 */
int Session::Bind(const struct sockaddr *addr, socklen_t addrlen)
{
  if (fd_ == -1)
    return -1;
  if (addr == nullptr)
    return -1;
  return bind(fd_, addr, addrlen);
}

/**
 * @brief 在一个已经绑定了地址的socket上侦听
 * @param backlog 侦听队列的最大长度；也就是等待处理的连接请求的最大数量
 */
int Session::Listen(int backlog)
{
  if (fd_ == -1)
    return -1;
  return listen(fd_, backlog);
}

/**
 * @brief 在指定的socket上接受一个连接请求
 * @param addr：建立连接的地址结构
 * @param addrlen：地址结构的大小，注意这里参数是一个指针，与connect()不同
 * 调用这个函数时，addr和addrlen在调用成功后将被填写好对端的地址和端口信息，所以在调用前最好将其清0；
 * 如果我们并不关心对端的地址信息，这两个参数其实也可以为NULL；
 */
auto Session::Accept(struct sockaddr *addr, socklen_t *addrlen) -> SessionPtr
{
  if (fd_ == -1)
    return nullptr;
  // int sock_fd = accept4(fd_, addr, addrlen, SOCK_NONBLOCK);
  int sock_fd = accept(fd_, NULL, NULL);
  while (sock_fd == -1 && (errno == EAGAIN || errno == EWOULDBLOCK))
  {
    // poll_handler_->Block(-1, true);
    sock_fd = accept(fd_, NULL, NULL);
    // sock_fd = accept4(fd_, addr, addrlen, SOCK_NONBLOCK);
  }
  if (sock_fd == -1)
  {
    return nullptr;
  }
  return std::make_shared<Session>(sock_fd);
}

/**
 * @brief 创建与指定地址的连接；
 * @param addr：建立连接的地址结构
 * @param addrlen：地址结构的大小
 */
int Session::Connect(const struct sockaddr *addr, socklen_t addrlen)
{
  if (fd_ == -1)
    return -1;

  int optval;
  socklen_t optlen = sizeof(optval);
  int res = connect(fd_, addr, addrlen);
  if (res == -1 && errno == EINPROGRESS)
  {
    // poll_handler_->Block(-1, false);
    getsockopt(fd_, SOL_SOCKET, SO_ERROR, reinterpret_cast<void *>(&optval),
               &optlen);
    if (optval == 0)
    {
      res = 0;
    }
    else
    {
      errno = optval;
    }
  }
  return res;
}

int Session::Close()
{
  if (fd_ == -1)
    return -1;
  // poll_handler_->Unblock();
  int res = close(fd_);
  fd_ = -1;
  return res;
}

ssize_t Session::Recv(void *buf, size_t len, int flags, int timeout_ms)
{
  if (buf == nullptr)
    return -1;
  if (fd_ == -1)
    return -1;

  ssize_t nbytes = recv(fd_, buf, len, flags);
  if (timeout_ms == 0)
  {
    return nbytes;
  }
  // while (nbytes == -1 && (errno == EAGAIN || errno == EWOULDBLOCK))
  // {
  //   nbytes = recv(fd_, buf, len, flags);
  //   if (timeout_ms > 0)
  //   {
  //     break;
  //   }
  // }
  return nbytes;
}

ssize_t Session::RecvFrom(void *buf, size_t len, int flags,
                          struct sockaddr *src_addr, socklen_t *addrlen,
                          int timeout_ms)
{
  if (buf == nullptr)
    return -1;
  if (fd_ == -1)
    return -1;

  ssize_t nbytes = recvfrom(fd_, buf, len, flags, src_addr, addrlen);
  if (timeout_ms == 0)
  {
    return nbytes;
  }
  // while (nbytes == -1 && (errno == EAGAIN || errno == EWOULDBLOCK))
  // {
  //   nbytes = recvfrom(fd_, buf, len, flags, src_addr, addrlen);
  //   if (timeout_ms > 0)
  //   {
  //     break;
  //   }
  // }
  return nbytes;
}

ssize_t Session::Send(const void *buf, size_t len, int flags, int timeout_ms)
{
  if (buf == nullptr)
    return -1;
  if (fd_ == -1)
    return -1;

  ssize_t nbytes = send(fd_, buf, len, flags);
  if (timeout_ms == 0)
  {
    return nbytes;
  }
  // while ((nbytes == -1) && (errno == EAGAIN || errno == EWOULDBLOCK))
  // {
  //     nbytes = send(fd_, buf, len, flags);
  //   if (timeout_ms > 0)
  //   {
  //     break;
  //   }
  // }
  return nbytes;
}

ssize_t Session::SendTo(const void *buf, size_t len, int flags,
                        const struct sockaddr *dest_addr, socklen_t addrlen,
                        int timeout_ms)
{
  if (buf == nullptr)
    return -1;
  if (dest_addr == nullptr)
    return -1;
  if (fd_ == -1)
    return -1;

  ssize_t nbytes = sendto(fd_, buf, len, flags, dest_addr, addrlen);
  if (timeout_ms == 0)
  {
    return nbytes;
  }
  // while ((nbytes == -1) && (errno == EAGAIN || errno == EWOULDBLOCK))
  // {
  //     nbytes = sendto(fd_, buf, len, flags, dest_addr, addrlen);
  //   if (timeout_ms > 0)
  //   {
  //     break;
  //   }
  // }
  return nbytes;
}

/**
 * @brief 从指定文件描述符中读出内容到缓冲区中
 * @param buf：存放读出内容的缓冲区
 * @param count：buf的大小
 */
ssize_t Session::Read(void *buf, size_t count, int timeout_ms)
{
  if (buf == nullptr)
    return -1;
  if (fd_ == -1)
    return -1;

  ssize_t nbytes = read(fd_, buf, count);
  if (timeout_ms == 0)
  {
    return nbytes;
  }
  // while ((nbytes == -1) && (errno == EAGAIN || errno == EWOULDBLOCK))
  // {
  //     nbytes = read(fd_, buf, count);
  //   if (timeout_ms > 0)
  //   {
  //     break;
  //   }
  // }
  return nbytes;
}

/**
 * @brief 将缓冲区内容写入指定文件描述符中
 * @param buf：存放读出内容的缓冲区
 * @param count：buf的大小
 */
ssize_t Session::Write(const void *buf, size_t count, int timeout_ms)
{
  if (buf == nullptr)
    return -1;
  if (fd_ == -1)
    return -1;

  ssize_t nbytes = write(fd_, buf, count);
  if (timeout_ms == 0)
  {
    return nbytes;
  }
  // while ((nbytes == -1) && (errno == EAGAIN || errno == EWOULDBLOCK))
  // {
  //   if (poll_handler_->Block(timeout_ms, false))
  //   {
  //     nbytes = write(fd_, buf, count);
  //   }
  //   if (timeout_ms > 0)
  //   {
  //     break;
  //   }
  // }
  return nbytes;
}

int Session::FetchIfaceIndex(const char *iface)
{
  // fill iface name to struct ifreq
  struct ifreq ifr;
  memset(&ifr, 0x00, sizeof(ifr));
  strncpy(ifr.ifr_name, iface, sizeof(ifr.ifr_name));
  // call ioctl system call to fetch iface index
  if (ioctl(fd_, SIOCGIFINDEX, &ifr) == -1)
  {
    return -1;
  }
  return ifr.ifr_ifindex;
}

END_NS_ZF_DRIVER_IO