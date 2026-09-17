// #include <cstdio>

// int main(int argc, char ** argv)
// {
//   (void) argc;
//   (void) argv;

//   printf("hello world zf_hal_canmsg_parser package\n");
//   return 0;
// }

#include "zf_hal_canmsg_parser/hal_canmsg_parser.h"

#include <cmath>
#include <algorithm>

#include "zf_hal_can_driver/hal_can_id.h"

BEGIN_NS_ZF_DRIVER_CANBUS

CanMsgParser::CanMsgParser()
{
}

CanMsgParser::~CanMsgParser()
{
}

std::string CanMsgParser::ExtractFileName(const std::string &filepath, bool withExtension)
{
    // Example file path
    // std::string filepath = "/path/to/myfile.txt";

    // Extract the filename from the file path
    size_t last_slash = filepath.find_last_of("/\\");
    std::string filename = (last_slash == std::string::npos) ? filepath : filepath.substr(last_slash + 1);
    // Remove the extension from the filename
    if (withExtension)
        return filename;
    size_t last_dot = filename.find_last_of(".");
    if (last_dot != std::string::npos)
    {
        filename = filename.substr(0, last_dot);
    }
    return filename;
}

bool CanMsgParser::CheckValid(const std::string &dbcName, const std::string &pgn, const std::string &sigName)
{
    auto ite = m_mapDBCParser.find(dbcName);
    // std::string excepText = "";
    if (ite == m_mapDBCParser.end())
    {
        // excepText = "Cannot find DBCParser with dbc name: " + dbcName;
        // throw std::out_of_range(excepText);
        return false;
    }
    if (!pgn.empty())
    {
        if (!m_mapDBCParser.at(dbcName)->CheckPGN(pgn))
        {
            // excepText = "Cannot find DBCParser with dbc name: " + dbcName + ", pgn: " + pgn;
            // throw std::out_of_range(excepText);
            return false;
        }
        if (!sigName.empty())
        {
            auto msg = m_mapDBCParser.at(dbcName)->at(pgn);
            if (!msg.CheckSignal(sigName))
            {
                // excepText = "Cannot find DBCParser with dbc name: " + dbcName + ", sig name: " + sigName;
                // throw std::out_of_range(excepText);
                return false;
            }
        }
    }
    return true;
}

std::string CanMsgParser::AddDBCParser(const std::string &filePath)
{
    auto name = ExtractFileName(filePath);
    AddDBCParser(name, filePath);
    return name;
}

void CanMsgParser::AddDBCParser(const std::string &name, const std::string &filePath)
{
    m_mapDBCParser.insert(std::make_pair(name, std::make_shared<DBCIterator>(filePath)));
}

std::shared_ptr<DBCIterator> CanMsgParser::GetDBCParser(const std::string &name)
{
    return m_mapDBCParser.at(name);
}

void CanMsgParser::PrintDBCParser(const std::string &name)
{

    for (auto message : *m_mapDBCParser.at(name).get())
    {
        // std::cout << message.second.getName() << " " << std::hex << std::showbase << message.second.getId() << std::endl;
        std::cout << message.second.getName() << " 0x" << message.second.getPGN() << std::endl;
        // for (const auto &sig : message.second)
        // {
        //     std::cout << sig.second.getName() << " ---" << sig.second.getLsb().value << " ---" << sig.second.getMsb().value << std::endl;
        // }
        std::cout << std::endl;
    }
}

void CanMsgParser::RemoveDBCParser(const std::string &name)
{
    auto ite = m_mapDBCParser.find(name);
    if (ite == m_mapDBCParser.end())
    {
        LOG_ERROR() << "Cannot remove DBCParser with name " << name.c_str();
    }
    m_mapDBCParser.erase(ite);
}

NS_ZF::ErrorCode CanMsgParser::Parse(SignalPPMap &sigVals, const CanDataInfo &info)
{
    sigVals.clear();
    switch (info.GetPGNType())
    {
    case CanDataInfo::PGNType::BROADCAST:
    case CanDataInfo::PGNType::CUSTOM:
    case CanDataInfo::PGNType::P2P:
        return ParseImpl(sigVals, info);
    case CanDataInfo::PGNType::TPCM:
    {
        TP_CM cm;
        cm.from_ecu_address = info.GetEcuAddr();
        ParseImpl(sigVals, info);
        // Parse(sigVals, info.GetData(), info.GetDBCName(), info.GetPGN(), "");
        cm.control_byte = sigVals["ControlByte"];
        cm.PGN_of_the_packeted_message = sigVals["PGNumber"];
        cm.number_of_packages = sigVals["TotalNumberOfPackets"];
        cm.total_message_size = sigVals["TotalMessageSize"];
        cm.isDone = false;
        m_mapTPCM[cm.from_ecu_address] = cm;
        // m_mapTPCM.insert(std::make_pair(data.from_ecu_address, data));
        return ErrorCode::CAN_PARSE_IGNORE;
    }
    case CanDataInfo::PGNType::TPDT:
    {
        TP_DT dt;
        dt.from_ecu_address = info.GetEcuAddr();
        // Parse(sigVals, info.GetData(), dbcName, info.GetPGN(), "");
        // dt.sequence_number = sigVals["SequenceNumber"];
        auto tDat = info.GetData();
        dt.sequence_number = tDat.front();
        tDat.erase(tDat.begin());
        dt.data = tDat;
        m_mapTPCM[dt.from_ecu_address].data.push_back(dt);
        std::sort(m_mapTPCM[dt.from_ecu_address].data.begin(),
                  m_mapTPCM[dt.from_ecu_address].data.end());
        // std::sort(data.begin(), data.end(), std::greater<TP_DT>());//less
        if (m_mapTPCM[dt.from_ecu_address].data.size() ==
            m_mapTPCM[dt.from_ecu_address].number_of_packages)
        {
            m_mapTPCM[dt.from_ecu_address].isDone = true;
            auto pgn = Byte::PGNFromCanId(m_mapTPCM[dt.from_ecu_address].PGN_of_the_packeted_message);
            std::vector<uint8_t> mergedData;
            MergeTPCMData(mergedData, m_mapTPCM[dt.from_ecu_address]);

            auto errCode = Parse(sigVals, mergedData, info.GetDBCName(), pgn, "");
            // sigVals[gk_keyTPDTpgn] = m_mapTPCM[dt.from_ecu_address].PGN_of_the_packeted_message;
            sigVals[gk_keyTPDTpgn] = CanId::GenerateCanId((info.GetCanID() >> 24),
                                                          m_mapTPCM[dt.from_ecu_address].PGN_of_the_packeted_message,
                                                          dt.from_ecu_address);

            if (errCode == ErrorCode::CAN_PARSE_OUT_OF_BOUND)
            {
                info.PrintInfo();
                for (uint8_t i = 0; i < mergedData.size(); ++i)
                {
                    std::cout << Byte::Int2Hex(mergedData[i]) << " ";
                }
                LOG_DEBUG() << int(errCode) << ", PGN: " << pgn << ", gk_keyTPDTpgn: " << sigVals[gk_keyTPDTpgn];
            }
            return errCode;
        }
        return ErrorCode::CAN_PARSE_IGNORE;
    }
    default:
        break;
    }
    return ErrorCode::OK;
}

NS_ZF::ErrorCode CanMsgParser::Parse(SignalPPMap &sigVals, const std::vector<CanDataType> &dat, DBCSignalOptions &dbcOpt)
{
    if (dbcOpt.dbcName.empty() || (!dbcOpt.address))
    {
        return ErrorCode::CAN_PARSE_INVALID_NAME;
    }
    if (dbcOpt.sigName.empty())
    {
        Parse(sigVals, dat, dbcOpt.dbcName, dbcOpt.address);
    }
    else
    {
        dbcOpt.value = Parse(dat, dbcOpt.dbcName, dbcOpt.address, dbcOpt.sigName);
    }
    return ErrorCode::OK;
}

ErrorCode CanMsgParser::Parse(const std::vector<CanDataType> &dat, DBCSignalOptions &dbcOpt)
{
    if (dbcOpt.dbcName.empty() || (!dbcOpt.address) || dbcOpt.sigName.empty())
    {
        return ErrorCode::CAN_PARSE_INVALID_NAME;
    }
    dbcOpt.value = Parse(dat, dbcOpt.dbcName, dbcOpt.address, dbcOpt.sigName);
    return ErrorCode::OK;
}

ErrorCode CanMsgParser::Parse(SignalPPMap &sigVals, const std::vector<CanDataType> &dat, const std::string &dbcName, const uint32_t &address)
{
    CanDataInfo info(address, dat, dbcName);
    return ParseImpl(sigVals, info);
}

double CanMsgParser::Parse(const std::vector<CanDataType> &dat,
                           const std::string &dbcName,
                           const uint32_t &address,
                           const std::string &sigName)
{
    CanDataInfo info(address, dat, dbcName);
    info.SetSigNames({sigName});
    SignalPPMap sigVals;
    if (ParseImpl(sigVals, info) == ErrorCode::OK)
    {
        return sigVals[sigName];
    }
    return NULL;
}

NS_ZF::ErrorCode CanMsgParser::Parse(SignalPPMap &sigVals, const std::vector<CanDataType> &dat,
                                     const std::string &dbcName, const std::string &pgn, const std::string &ecuId)
{
    if (!CheckValid(dbcName, pgn))
    {
        auto excepText = "Cannot find DBCParser with dbc name: " + dbcName + ", pgn: " + pgn;
        // throw std::out_of_range();
        LOG_ERROR() << excepText.c_str();
        return ErrorCode::CAN_PARSE_INVALID_NAME;
    }
    const auto &msgs = m_mapDBCParser.at(dbcName)->at(pgn, ecuId);
    if (!msgs.getSize())
    {
        return ErrorCode::CAN_PARSE_INVALID_NAME;
    }
    sigVals.clear();
    NS_ZF::ErrorCode errCode = ErrorCode::OK;
    for (const auto &sigMap : msgs)
    {
        const auto &sig = sigMap.second;
        double val;
        if (GetValue<CanDataType>(val, dat, sig) == ErrorCode::CAN_PARSE_OUT_OF_BOUND)
        {
            errCode = ErrorCode::CAN_PARSE_OUT_OF_BOUND;
        }
        GetValue<CanDataType>(val, dat, sig);
        sigVals.insert(std::make_pair(sig.getName(), val));
    }
    return errCode;
}

NS_ZF::ErrorCode CanMsgParser::ParseImpl(SignalPPMap &sigVals, const CanDataInfo &info)
{
    auto dbcName = info.GetDBCName();
    auto pgn = info.GetPGN();
    if (!CheckValid(dbcName, pgn))
    {
        auto excepText = "Cannot find DBCParser with dbc name: " + dbcName + ", pgn: " + pgn;
        // throw std::out_of_range();
        LOG_ERROR() << excepText.c_str();
        return ErrorCode::CAN_PARSE_INVALID_NAME;
    }
    Message msgs;
    switch (info.GetPGNType())
    {
    case CanDataInfo::PGNType::BROADCAST:
    case CanDataInfo::PGNType::CUSTOM:
    case CanDataInfo::PGNType::P2P:
    {
        msgs = m_mapDBCParser.at(dbcName)->at(pgn, info.GetEcuAddr());
        // msgs = m_mapDBCParser.at(dbcName)->at(pgn, "007F");
        break;
    }
    case CanDataInfo::PGNType::TPCM:
    case CanDataInfo::PGNType::TPDT:
    {
        msgs = m_mapDBCParser.at(dbcName)->at(pgn, "");
        break;
    }
    default:
    {
        msgs = m_mapDBCParser.at(dbcName)->at(pgn, info.GetEcuAddr());
        break;
    }
    }
    if (!msgs.getSize())
    {
        return ErrorCode::CAN_PARSE_INVALID_NAME;
    }
    // LOG_INFO() << "msgs name :" << msgs.getName();
    sigVals.clear();
    auto sigNames = info.GetSigNames();
    if (sigNames.empty())
    {
        for (const auto &sigMap : msgs)
        {
            const auto &sig = sigMap.second;
            double val;
            auto errCode = GetValue<CanDataType>(val, info.GetData(), sig, info.GetCanID(), info.IsIgnoreCounter(), info.IsIgnoreChecksum());
            // LOG_WARN() << "CAN_PARSE_CHECK_FAILED: " << sig.getName().c_str();

            if (errCode == ErrorCode::CAN_PARSE_CHECK_FAILED)
            {
                LOG_WARN() << "CAN_PARSE_CHECK_FAILED: " << msgs.getName().c_str() << ", " << sig.getName().c_str();
                return errCode;
            }
            else
            {
                if (errCode == ErrorCode::CAN_PARSE_OUT_OF_BOUND)
                {
                    info.PrintInfo();
                }
                sigVals.insert(std::make_pair(sig.getName(), val));
            }
        }
    }
    else
    {
        for (const auto &sigName : sigNames)
        {
            if (msgs.CheckSignal(sigName))
            {
                const auto &sig = msgs.at(sigName);
                double val;
                auto errCode = GetValue<CanDataType>(val, info.GetData(), sig, info.GetCanID(), info.IsIgnoreCounter(), info.IsIgnoreChecksum());
                if (errCode == ErrorCode::CAN_PARSE_CHECK_FAILED)
                {
                    LOG_WARN() << "CAN_PARSE_CHECK_FAILED: " << msgs.getName().c_str() << ", " << sig.getName().c_str();
                    return errCode;
                }
                else
                {
                    if (errCode == ErrorCode::CAN_PARSE_OUT_OF_BOUND)
                    {
                        info.PrintInfo();
                    }
                    sigVals.insert(std::make_pair(sig.getName(), val));
                }
            }
        }
    }
    return ErrorCode::OK;
}

NS_ZF::ErrorCode CanMsgParser::Pack(std::vector<CanMsgParser::CanDataType> &data, const SignalPPMap &signals, const std::string &dbcName, uint32_t address)
{
    auto pgn = Byte::PGNFromCanId(address);
    auto ecuId = Byte::EcuFromCanId(address);
    auto msgs = m_mapDBCParser.at(dbcName)->at(pgn, ecuId);

    std::vector<CanDataType> ret(msgs.getDlc(), 0x0); // defult is 0
    if (msgs.getSizeValid() < signals.size())
    {
        return ErrorCode::CAN_PARSE_INVALID_NAME;
    }

    // set all values for all given signal/value pairs
    for (const auto &sigval : signals)
    {
        if (!msgs.CheckSignal(sigval.first))
        {
            // TODO: do something more here. invalid flag like CANParser?
            auto excepText = "undefined signal with dbc name: " + dbcName + ", pgn: " + pgn + ", sig name: " + sigval.first;
            LOG_ERROR() << excepText.c_str();
            continue;
        }
        const auto sig = msgs.at(sigval.first);

        int64_t ival = (int64_t)(std::round((sigval.second - sig.getOffset()) / sig.getFactor()));
        if (ival < 0)
        {
            ival = (1ULL << sig.getLength()) + ival;
        }
        SetValue(ret, sig, ival);
        // std::cout << sig.getName() << "and " << ival << "  ";
        // for (uint8_t i = 0; i < ret.size(); ++i)
        // {
        //     std::cout << Byte::Int2Hex(ret[i]) << " ";
        // }
        // LOG_DEBUG();
    }

    // std::vector<CanDataType> retOri = ret;
    // set message counter
    if (msgs.HasCounter())
    {
        const auto &sig = msgs.getCounterSig();
        if (m_mapPackCounters.find(address) == m_mapPackCounters.end())
        {
            m_mapPackCounters[address] = 0;
        }
        SetValue(ret, sig, m_mapPackCounters[address]);
        if (!pgn.compare(gk_pgnTSC1))
        {
            m_mapPackCounters[address] = (m_mapPackCounters[address] + 1) % 8;
        }
        else
        {
            m_mapPackCounters[address] = (m_mapPackCounters[address] + 1) % (1 << sig.getLength());
        }
    }

    // set message checksum
    if (msgs.HasChecksum())
    {
        const auto &sig = msgs.getChecksumSig();
        // ret[0] = 0x30;
        uint32_t checksum = CanDataInfo::CalChecksum(address, ret);
        SetValue(ret, sig, checksum);
        // LOG_INFO() << pgn.c_str() << " m_mapPackCounters: " << m_mapPackCounters[address] << " checksum: " << checksum;
    }
    // for (uint8_t i = 0; i < ret.size(); ++i)
    // {
    //     std::cout << Byte::Int2Hex(ret[i]) << " ";
    // }
    // LOG_DEBUG();

    data.swap(ret);

    return ErrorCode::OK;
}

int64_t CanMsgParser::GetRawValue(const std::vector<uint8_t> &data, const Signal &sig)
{
    int64_t ret = 0;
    // int i = sig.getMsb() / 8;
    int i = sig.getMsb().x;
    int bits = sig.getLength();
    auto dataLen = static_cast<int>(data.size());
    // From msb to lsb
    while (i >= 0 && i < dataLen && bits > 0)
    {
        // int lsb = (int)(sig.lsb / 8) == i ? sig.lsb : i * 8;
        // int msb = (int)(sig.msb / 8) == i ? sig.msb : (i + 1) * 8 - 1;
        int lsb = sig.getLsb().x == i ? sig.getLsb().value : i * 8;
        int msb = sig.getMsb().x == i ? sig.getMsb().value : (i + 1) * 8 - 1;
        int size = msb - lsb + 1;

        uint64_t d = (data[i] >> (lsb - (i * 8))) & ((1ULL << size) - 1);
        ret |= d << (bits - size);

        bits -= size;
        i = (sig.getByteOrder() == ByteOrder::INTEL) ? i - 1 : i + 1;
        // i = sig.is_little_endian ? i - 1 : i + 1;
    }
    return ret;
}

void CanMsgParser::SetValue(std::vector<uint8_t> &data, const Signal &sig, int64_t ival)
{
    //   int i = sig.lsb / 8;
    int i = sig.getLsb().x;
    int bits = sig.getLength();
    if (bits < 64)
    {
        ival &= ((1ULL << bits) - 1);
    }
    auto dataLen = static_cast<int>(data.size());
    // From lsb to msb
    while (i >= 0 && i < dataLen && bits > 0)
    {
        // int shift = (int)(sig.lsb / 8) == i ? sig.lsb % 8 : 0;
        int shift = sig.getLsb().x == i ? sig.getLsb().y : 0;
        int size = std::min(bits, 8 - shift);
        // For 5, 3, (1ULL << size) - 1): 11111; << shift 11111000; ~: 00000111, thus the first size len is 0 now.
        data[i] &= ~(((1ULL << size) - 1) << shift);
        data[i] |= (ival & ((1ULL << size) - 1)) << shift;
        bits -= size;
        ival >>= size;
        i = (sig.getByteOrder() == ByteOrder::INTEL) ? i + 1 : i - 1;
    }
}

int64_t CanMsgParser::GetRawValue(const std::vector<Byte> &data, const Signal &sig)
{
    int64_t ret = 0;
    // int i = sig.getMsb() / 8;
    int i = sig.getMsb().x;
    int bits = sig.getLength();
    auto dataLen = static_cast<int>(data.size());
    // From msb to lsb
    while (i >= 0 && i < dataLen && bits > 0)
    {
        // int lsb = (int)(sig.lsb / 8) == i ? sig.lsb : i * 8;
        // int msb = (int)(sig.msb / 8) == i ? sig.msb : (i + 1) * 8 - 1;
        int lsb = sig.getLsb().x == i ? sig.getLsb().y : 0; // start col
        int msb = sig.getMsb().x == i ? sig.getMsb().y : 7; // end col
        int size = msb - lsb + 1;

        uint64_t d = data[i].get_byte(lsb, size);
        ret |= d << (bits - size);

        bits -= size;
        i = (sig.getByteOrder() == ByteOrder::INTEL) ? i - 1 : i + 1;
        // i = sig.is_little_endian ? i - 1 : i + 1;
    }
    return ret;
}

void CanMsgParser::SetValue(std::vector<Byte> &data, const Signal &sig, int64_t ival)
{
    //   int i = sig.lsb / 8;
    int i = sig.getLsb().x;
    int bits = sig.getLength();
    if (bits < 64)
    {
        ival &= ((1ULL << bits) - 1);
    }
    auto dataLen = static_cast<int>(data.size());
    // From lsb to msb
    while (i >= 0 && i < dataLen && bits > 0)
    {
        // int shift = (int)(sig.lsb / 8) == i ? sig.lsb % 8 : 0;
        int shift = sig.getLsb().x == i ? sig.getLsb().y : 0; // start col
        int size = std::min(bits, 8 - shift);
        // For 5, 3, (1ULL << size) - 1): 11111; << shift 11111000; ~: 00000111, thus the first size len is 0 now.
        data[i].set_value(ival, shift, size);
        bits -= size;
        ival >>= size;
        i = (sig.getByteOrder() == ByteOrder::INTEL) ? i + 1 : i - 1;
    }
}

void CanMsgParser::MergeTPCMData(std::vector<uint8_t> &mergedData, const TP_CM &data)
{
    mergedData.clear();
    for (auto &tdat : data.data)
    {
        mergedData.insert(mergedData.end(), tdat.data.begin(), tdat.data.end());
    }
    auto msgSize = data.total_message_size;
    mergedData.resize(msgSize);
}

bool CanMsgParser::UpdateCounterGeneric(uint32_t addr, int64_t cnt, int cnt_size)
{
    if (m_mapParserCounters.find(addr) == m_mapPackCounters.end())
    {
        m_mapParserCounters[addr] = cnt;
        m_mapParserCountersFail[addr] = 0;
        return true;
    }
    uint8_t old_counter = m_mapParserCounters[addr];
    m_mapParserCounters[addr] = cnt;
    if (((old_counter + 1) & ((1 << cnt_size) - 1)) != cnt)
    {
        m_mapParserCountersFail[addr] += 1;
        if (m_mapParserCountersFail[addr] >= 1)
        {
            LOG_WARN() << "COUNTER FAIL: 0x" << Byte::Int2Hex(addr) << " #" << int(old_counter) << "--" << cnt;
        }
        if (m_mapParserCountersFail[addr] >= 5) // MAX_BAD_COUNTER
        {
            return false;
        }
    }
    else if (m_mapParserCountersFail[addr] > 0)
    {
        m_mapParserCountersFail[addr] -= 1;
    }
    return true;
}

END_NS_ZF_DRIVER_CANBUS