// #include <cstdio>

// int main(int argc, char ** argv)
// {
//   (void) argc;
//   (void) argv;

//   printf("hello world zf_io package\n");
//   return 0;
// }
#include "zf_io/zf_io_ethernet.h"
#include "zf_global/in/zf_framework_global.h"
#include <signal.h>
#include <cstring>
#include <thread>
#include <sys/mman.h> // for shm_open

using namespace zf::driver::io;

void client_sigint(int signum)
{
  fprintf(stdout, "Got ctrl+c signal in pid = %d.\n", getpid());
  shm_unlink(SHARED_MEMORY_KEY_TC397);
  exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[])
{
  signal(SIGINT, client_sigint);

  EthernetClient serv;
  serv.Open();
  serv.LoopReceive();

  // CAN_HDR can_data;
  // can_data.ip_ver = 5;
  // can_data.ip_len = 4;
  // can_data.ip_tos = 0X5A;
  // can_data.canfd_flag = 1;
  // can_data.canId = 4;
  // can_data.msg_id = 0x68F;
  // can_data.config_data_flag = 1;
  // can_data.length = 8;
  // char data[8] = {0xA0, 0xB0, 0xC0, 0xD0, 0xFE, 0xED, 0x00, 0x00};
  // memcpy(can_data.data, &data, 8);

  // serv.Send(can_data);
  while (true)
  {
    // Your node logic here

    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Pause for 1 second
  }

  return 0;
}