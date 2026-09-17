/*
 * main.cpp
 *
 *  Created on: 04.10.2013
 *      Author: downtimes
 */

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <iomanip>
#include <bitset>

#include "zf_hal_canmsg_parser/dbc_iterator.h"
#include "zf_hal_can_driver/byte.h"

const std::string usage = ""
                          "This parser is meant to be used via CLI at the moment\n"
                          "	./parser <FILE>\n"
                          "	\n"
                          "example:\n"
                          "	./parser /home/downtimes/kit13.dbc\n";

// template <typename T>
// std::string Id2PGN(T id, size_t width = sizeof(T) * 2)
// {
//     std::stringstream ssHex;
//     ssHex << std::uppercase << std::setfill('0') << std::setw(width) << std::hex << (id | 0);
//     if (ssHex.str().length() >= 6)
//     {
//         return ssHex.str().substr(2, 4);
//     }
//     else
//     {
//         return ssHex.str();
//     }
// }

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cout << usage << std::endl;
        return 0;
    }
    // std::vector<uint32_t> canId = {
    //     0x0CF02FA0,
    //     0x10FE6F2A,
    //     0x18FF30A0,
    //     0x1CECFFA0,
    //     0x0C040B2A,
    //     0x0C0A002A};
    std::vector<uint32_t> canId = {
        0x65D,
        0x2C1,
        0x510,
        0x551,
        0x718};
    std::vector<std::string> canIdStr;
    for (const auto &id : canId)
    {
        canIdStr.push_back(zf::driver::canbus::Byte::PGNFromCanId(id));
        std::cout << " 0x" << canIdStr.back().c_str() << ", " << std::endl;
    }

    // bitset<8> dataBits(msg->data.at(i));
    // bitString = dataBits.to_string();

    try
    {

        zf::driver::canbus::DBCIterator dbc(argv[1]);
        // DBCIterator dbc("./test_err.dbc");
        for (auto message : dbc)
        {
            // std::cout << message.second.getName() << " " << std::hex << std::showbase << message.second.getId() << std::endl;
            // std::cout << message.second.getName() << " 0x" << message.second.getIdHex() << std::endl;
            for (const auto &id : canIdStr)
            {
                if (!message.second.getPGN().compare(id))
                {
                    std::cout << message.second.getName() << " 0x" << message.second.getPGN()
                              << " 0x" << id << std::endl;
                }
            }
            // for(auto &sigMap : message.second) {
            //     auto sig = sigMap.second;
            //     std::cout << "Signal: " << sig.getName() << "  ";
            //     //std::cout << "To: ";
            //     //for (auto to : sig.getTo()) {
            //     //    std::cout << to << ", ";
            //     //}
            //     //std::cout << sig.getStartbit() << "," << sig.getLength() << std::endl;
            //     //std::cout << "(" << sig.getFactor() << ", " << sig.getOffset() << ")" << std::endl;
            //     //std::cout << "[" << sig.getMinimum() << "," << sig.getMaximum() << "]" << std::endl;
            //     //if (sig.getMultiplexor() == Multiplexor::MULTIPLEXED) {
            //     //    std::cout << "#" << sig.getMultiplexedNumber() << "#" << std::endl;
            //     //} else if (sig.getMultiplexor() == Multiplexor::MULTIPLEXOR) {
            //     //    std::cout << "+Multiplexor+" << std::endl;
            //     //}
            //     std::cout << std::endl;
            // };
        }
    }
    catch (std::invalid_argument &ex)
    {
        std::cout << ex.what() << std::endl;
    }

    //	return 0;
}

// #include <bitset>
// #include <dirent.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <sys/stat.h>
// size_t countDiskUsage(const char* pathname)
// {
//   if (pathname == NULL) {
//     printf("Erorr: pathname is NULL\n");
//   }

//   struct stat stats;

//   if (lstat(pathname, &stats) == 0) {
//     if (S_ISREG(stats.st_mode)){
//       return stats.st_size;
//     }
//   } else {
//     perror("lstat\n");
//   }

//   DIR* dir = opendir(pathname);

//   if (dir == NULL) {
//     perror("Error");
//     return 0;
//   }

//   struct dirent *dirEntry;
//   size_t totalSize = 4096;

//   for (dirEntry = readdir(dir); dirEntry != NULL; dirEntry =   readdir(dir)) {
//     long pathLength = sizeof(char) * (strlen(pathname) + strlen(dirEntry->d_name) + 2);
//     char* name = (char*)malloc(pathLength);
//     strcpy(name, pathname);
//     strcpy(name + strlen(pathname), "/");
//     strcpy(name + strlen(pathname) + 1, dirEntry->d_name);

//     if (dirEntry->d_type == DT_DIR) {
//       if (strcmp(dirEntry->d_name, ".") != 0 && strcmp(dirEntry->d_name, "..") != 0) {
//         totalSize += countDiskUsage(name);
//       }
//     } else {
//       int status = lstat(name, &stats);
//       if (status == 0) {
//         totalSize += stats.st_size;
//       } else {
//         perror("lstat\n");
//       }
//     }
//     free(name);
//   }

//   closedir(dir);

//   return totalSize;
// }


// int main(int argc, char *argv[])
// {
//     std::string name = argv[1];
//     size_t size = countDiskUsage(argv[1]);
//     LOG_DEBUG() << "name is: " << argv[1] << ", and size " << size;
//     return 0;

// }