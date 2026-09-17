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

// #include "zf_hal_canmsg_parser/dbc_iterator.h"
#include "zf_hal_canmsg_parser/hal_canmsg_parser.h"
#include "zf_hal_can_driver/byte.h"

const std::string usage = ""
                          "This parser is meant to be used via CLI at the moment\n"
                          "	./dbc_parser_test <FILE>\n"
                          "	\n"
                          "example:\n"
                          "	./dbc_parser_test /home/downtimes/VehicleCAN.dbc\n";

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

using namespace zf::driver::canbus;
using namespace zf;

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
    // std::vector<uint32_t> canId = {
    //     0x65D,
    //     0x2C1,
    //     0x510,
    //     0x551,
    //     0x718};
    // std::vector<std::string> canIdStr;
    // for (const auto &id : canId)
    // {
    //     canIdStr.push_back(zf::driver::canbus::Byte::PGNFromCanId(id));
    //     std::cout << " 0x" << canIdStr.back().c_str() << ", " << std::endl;
    // }

    // bitset<8> dataBits(msg->data.at(i));
    // bitString = dataBits.to_string();

    try
    {
        std::string dbcName = "Parser";

        zf::driver::canbus::DBCIterator dbc(argv[1]);
        // std::unique_ptr<zf::driver::canbus::CanMsgParser> msgParser = std::make_unique<zf::driver::canbus::CanMsgParser>();
        CanMsgParser::Instance().AddDBCParser(dbcName, argv[1]);
        // CanMsgParser::Instance().PrintDBCParser(dbcName);

        // std::vector<Byte> dat = {0xFF, 0xFF, 0xFF, 0xC6, 0x00, 0xFD, 0x30, 0xFF};
        // std::vector<uint8_t> dat = {0xFF, 0xFF, 0xFF, 0xC6, 0x00, 0xFD, 0x30, 0xFF};
        std::vector<uint8_t> dat = {0X00, 0x90, 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x08};
        // std::vector<uint8_t> dat = {0x0B, 0x7D , 0x1F , 0x56 , 0x36 , 0x72 , 0xC6 , 0xCD};
        // std::vector<uint8_t> dat = {0x03, 0x7D, 0x1F, 0x58, 0x25, 0x9B, 0x04, 0xB2};
        // // std::vector<Byte> datB;
        // // for (auto da: dat){
        // //     datB.emplace_back(&da);
        // // }

        CanMsgParser::SignalPPMap t;
        DBCSignalOptions dbcOpt;
        dbcOpt.dbcName = dbcName;
        // dbcOpt.address = 0x65D;
        dbcOpt.address = 0x18FFEF80;

        CanMsgParser::Instance().Parse(t, dat, dbcOpt);
        for (auto tv : t)
        {
            LOG_INFO() << tv.first.c_str() << " and " << tv.second;
        }

        // CanMsgParser::SignalPPMap t;
        CanMsgParser::SignalPPMap tPack;
        std::vector<uint8_t> data;


        // LOG_INFO() << "------------- verify EC00 EB00 begin-------------";
        // CanDataInfo info1(0x18ECFF00, {0x20, 0x28, 0x00, 0x06, 0xFF, 0xE3, 0xFE, 0x00}, dbcName);
        // CanDataInfo info2(0x18EBFF00, {0x01, 0xE0, 0x15, 0xB3, 0x30, 0x39, 0xC6, 0x00}, dbcName);
        // CanDataInfo info3(0x18EBFF00, {0x02, 0x19, 0xC0, 0x00, 0x1E, 0xD0, 0xE0, 0x2E}, dbcName);
        // CanDataInfo info4(0x18EBFF00, {0x03, 0xD3, 0x80, 0x3E, 0xFF, 0xFF, 0xF0, 0x0C}, dbcName);
        // CanDataInfo info5(0x18EBFF00, {0x04, 0xC0, 0x44, 0x3C, 0x37, 0xC8, 0x7D, 0xD0}, dbcName);
        // CanDataInfo info6(0x18EBFF00, {0x05, 0x80, 0x3E, 0xF4, 0x00, 0xFF, 0xFF, 0xFF}, dbcName);
        // CanDataInfo info7(0x18EBFF00, {0x06, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, dbcName);

        // std::vector<CanDataInfo> vecInfo;
        // vecInfo.push_back(info1);
        // vecInfo.push_back(info2);
        // vecInfo.push_back(info3);
        // vecInfo.push_back(info4);
        // vecInfo.push_back(info5);
        // vecInfo.push_back(info6);
        // vecInfo.push_back(info7);
        // for (const auto &info : vecInfo)
        // {

        //     auto errcode = CanMsgParser::Instance().Parse(t, info);
        //     LOG_INFO() << (int)errcode;
        //     for (auto tv : t)
        //     {
        //         LOG_INFO() << tv.first.c_str() << " and " << tv.second;
        //     }
        // }
        // // uint8_t high;
        // // uint8_t low;
        // // std::string result = "";
        // // auto id = 0x18ECFF00;
        // // if (id >= 65536) // current PGN: Form 8 bits to 24 bits
        // // {
        // //     high = static_cast<uint8_t>((id >> 16) & 0xFF); //& max value 0xFF for length 8, so limited length is 8
        // //     low = static_cast<uint8_t>((id >> 8) & 0xFF);
        // //     result += Byte::byte_to_hex(high);
        // //     if ((high & 0xF0) != 0xF0)
        // //     {
        // //         result += "00";
        // //     }
        // //     else
        // //     {
        // //         result += Byte::byte_to_hex(low);
        // //     }
        // // }
        // // LOG_INFO() << result.c_str();
        // LOG_INFO() << "------------- verify EC00 EB00 end-------------";
        // LOG_INFO();

        // LOG_INFO() << "------------- verify TSC1 begin-------------";
        // DBCSignalOptions dbcOpt;
        // dbcOpt.dbcName = dbcName;
        // dbcOpt.address = 0xC00100B;
        // // CanMsgParser::Instance().Parse(t, {0x30, 0xFF, 0xFF, 0x7D, 0xFF, 0xFF, 0xFF, 0x73}, dbcOpt);
        // CanMsgParser::Instance().Parse(t, {0x30, 0xFF, 0xFF, 0x7D, 0xFF, 0x0F, 0x00, 0x70}, dbcOpt);
        // for (auto tv : t)
        // {
        //     LOG_INFO() << tv.first.c_str() << " and " << tv.second;
        // }
        // LOG_INFO();

        // dbcOpt.address = 0xC000F0B;
        // CanMsgParser::Instance().Parse(t, {0x30, 0xFF, 0xFF, 0x7D, 0xFF, 0x0F, 0x00, 0x50}, dbcOpt);
        // // CanMsgParser::Instance().Parse(t, {0x30, 0xFF, 0xFF, 0x7D, 0xFF, 0xFF, 0xFF, 0x63}, dbcOpt);
        // for (auto tv : t)
        // {
        //     LOG_INFO() << tv.first.c_str() << " and " << tv.second;
        // }
        // tPack["EngOverrideCtrlMode"] = 0;
        // tPack["EngRqedSpeedCtrlConditions"] = 0;
        // tPack["EngRqedTorque_TorqueLimit"] = 0;
        // tPack["EngRqedSpeed_SpeedLimit"] = 8191.88;
        // tPack["EngineRequestedTorqueHiRes"] = 1.875;
        // tPack["ControlPurpose"] = 31;
        // tPack["OverrideCtrlModePriority"] = 3;
        // tPack["TransmissionRate"] = 7;
        // CanMsgParser::Instance().Pack(data, tPack, dbcName, 0xC00100B);
        // LOG_INFO();
        // CanMsgParser::Instance().Pack(data, tPack, dbcName, 0xC000F0B);
        // LOG_INFO() << "------------- verify TSC1 end-------------";

        // // // dbcOpt.address = 2349075327;
        // // // CanMsgParser::Instance().Parse(t, {0x02, 0x19, 0xC0, 0x00, 0x1E, 0xD0, 0xE0, 0x2E}, dbcOpt);

        // // // DBCSignalOptions dbcOpt;
        // // // dbcOpt.dbcName = dbcName;
        // // // dbcOpt.address = 0x18FEF121;
        // // // CanMsgParser::Instance().Parse(t, {0xF3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, dbcOpt);
        // // // for (auto tv : t)
        // // // {
        // // //     LOG_INFO() << tv.first.c_str() << " and " << tv.second;
        // // // }

        LOG_INFO();
        LOG_INFO() << "------------- verify XBR begin -------------";
        uint32_t XBR_addr = 0x0C040B80;
        CanDataInfo info(XBR_addr, {0x7f, 0x6d, 0x20, 0x00, 0x00, 0x00, 0x00, 0x54}, dbcName);
        // info.SetIgnoreChecksum(true);
        // info.SetIgnoreCounter(true);
        CanMsgParser::Instance().Parse(t, info);
        for (auto tv : t)
        {
            LOG_INFO() << tv.first.c_str() << " and " << tv.second;
        }
        LOG_INFO();

        tPack.clear();
        tPack["ExtlAccelerationDemand"] = -2;
        tPack["XBRCtrlMode"] = 2;
        tPack["XBREBIMode"] = 0; 
        tPack["XBRPriority"] = 0;
        tPack["XBR_urgency"] = 0;
        CanMsgParser::Instance().Pack(data, tPack, dbcName, XBR_addr);
        LOG_INFO() << "------------- verify XBR end-------------";

        auto c = CanDataInfo::CalChecksum(XBR_addr, {0x7f, 0x6d, 0x20, 0x00, 0x00, 0x00, 0x00, 0x54});
        LOG_INFO() << c;

        // // DBCIterator dbc("./test_err.dbc");
        // for (auto message : dbc)
        // {
        //     // std::cout << message.second.getName() << " " << std::hex << std::showbase << message.second.getId() << std::endl;
        //     // std::cout << message.second.getName() << " 0x" << message.second.getIdHex() << std::endl;
        //     for (const auto &id : canIdStr)
        //     {
        //         if (!message.second.getIdHex().compare(id))
        //         {
        //             std::cout << message.second.getName() << " 0x" << message.second.getIdHex()
        //                       << " 0x" << id << std::endl;
        //         }
        //     }
        //     // for(auto &sigMap : message.second) {
        //     //     auto sig = sigMap.second;
        //     //     std::cout << "Signal: " << sig.getName() << "  ";
        //     //     //std::cout << "To: ";
        //     //     //for (auto to : sig.getTo()) {
        //     //     //    std::cout << to << ", ";
        //     //     //}
        //     //     //std::cout << sig.getStartbit() << "," << sig.getLength() << std::endl;
        //     //     //std::cout << "(" << sig.getFactor() << ", " << sig.getOffset() << ")" << std::endl;
        //     //     //std::cout << "[" << sig.getMinimum() << "," << sig.getMaximum() << "]" << std::endl;
        //     //     //if (sig.getMultiplexor() == Multiplexor::MULTIPLEXED) {
        //     //     //    std::cout << "#" << sig.getMultiplexedNumber() << "#" << std::endl;
        //     //     //} else if (sig.getMultiplexor() == Multiplexor::MULTIPLEXOR) {
        //     //     //    std::cout << "+Multiplexor+" << std::endl;
        //     //     //}
        //     //     std::cout << std::endl;
        //     // };
        // }
    }
    catch (std::invalid_argument &ex)
    {
        std::cout << ex.what() << std::endl;
    }

    //	return 0;
}
