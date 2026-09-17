#include "zf_hal_canmsg_parser/hal_can_info.h"

#include <cmath>

#include "zf_hal_canmsg_parser/hal_can_pgn_define.h"
#include "zf_hal_can_driver/byte.h"

BEGIN_NS_ZF_DRIVER_CANBUS

CanDataInfo::CanDataInfo(uint32_t canId, std::vector<uint8_t> data, std::string dbcName)
    : m_id(canId), m_data(data), m_sDBCName(dbcName)
{
    Init();
}

CanDataInfo::~CanDataInfo()
{
}

uint32_t CanDataInfo::GetCanID() const
{
    return m_id;
}

CanDataInfo::PGNType CanDataInfo::GetPGNType() const
{
    return m_pgnType;
}

std::string CanDataInfo::GetPGN() const
{
    return m_sPGN;
}

std::string CanDataInfo::GetDBCName() const
{
    return m_sDBCName;
}

std::string CanDataInfo::GetEcuAddr() const
{
    return m_sEcuAddress;
}

std::vector<uint8_t> CanDataInfo::GetData() const
{
    return m_data;
}

void CanDataInfo::SetData(const std::vector<uint8_t> &data)
{
    m_data = data;
}

std::vector<std::string> CanDataInfo::GetSigNames() const
{
    return m_sSigNames;
}

void CanDataInfo::SetSigNames(const std::vector<std::string> &names)
{
    m_sSigNames = names;
}

void CanDataInfo::SetTimeStamp(const uint32_t &sec, const uint32_t &nanosec)
{
    m_stamp.tv_sec = sec;
    m_stamp.tv_usec = nanosec;
}

uint32_t CanDataInfo::CalChecksum(uint32_t address, const std::vector<uint8_t> &d)
{
    int s = 0;
    auto pgn = Byte::PGNFromCanId(address);
    // LOG_INFO() << address << " " << pgn.c_str();
    // bool extended = m_id > 0x7FF;
    while (address)
    {
        s += address & 0xFF;
        address >>= 8;
    }
    for (int i = 0; i < d.size() - 1; i++)
    {
        s += d[i];
    }
    if (!pgn.compare(gk_pgnTSC1)) // TSC1
    {
        s += (d.back() & 0xF);
        return (((s >> 6) & 0x3) + (s >> 3) + s) & 0x7;
    }
    else if (!pgn.compare(gk_pgnXBR) || !pgn.compare(gk_pgnAEBS2)) // XBR_DMC
    {
        s += (d.back() & 0xF);
        return ((s >> 4) + s) & 0xF;
    }
    return s & 0xFF;
}

bool CanDataInfo::CalChecksum(uint32_t val, uint32_t address, const std::vector<uint8_t>& d)
{
    int s = 0;
    auto pgn = Byte::PGNFromCanId(address);
    // LOG_INFO() << address << " " << pgn.c_str();
    // bool extended = m_id > 0x7FF;
    while (address)
    {
        s += address & 0xFF;
        address >>= 8;
    }
    for (int i = 0; i < d.size() - 1; i++)
    {
        s += d[i];
    }
    if (!pgn.compare(gk_pgnTSC1)) // TSC1
    {
        s += (d.back() & 0xF);
        return ((((s >> 6) & 0x3) + (s >> 3) + s) & 0x7) == val;
    }
    else if (!pgn.compare(gk_pgnXBR) || !pgn.compare(gk_pgnAEBS2)) // XBR_DMC or AEBS2_27(AC1000T)
    {
        s += (d.back() & 0xF);
        return (((s >> 4) + s) & 0xF) == val;
    }
    return true;
}

// uint32_t CanDataInfo::CalChecksum(uint32_t address, const std::vector<uint8_t> &d, uint8_t counter)
// {
//     int s = 0;
//     auto pgn = Byte::PGNFromCanId(address);
//     while (address)
//     {
//         s += address & 0xFF;
//         address >>= 8;
//     }
//     for (int i = 0; i < d.size(); i++)// add all
//     {
//         s += d[i];
//     }
//     s += (counter & 0xF);
//     if (!pgn.compare(gk_pgnTSC1)) // TSC1
//     {
//         return (((s >> 6) & 0x3) + (s >> 3) + s) & 0x7;
//     }
//     else if (!pgn.compare(gk_pgnXBR)) // XBR_DMC
//     {
//         return ((s >> 4) + s) & 0xF;
//     }
//     return s & 0xFF;
// }

uint32_t CanDataInfo::CalChecksum(uint8_t counter)
{
    int s = 0;
    // bool extended = m_id > 0x7FF;
    auto address = m_id;
    while (address)
    {
        s += address & 0xFF;
        address >>= 8;
    }
    for (int i = 0; i < m_data.size() - 1; i++)
    {
        s += m_data[i];
    }
    if (counter)
    {
        s += (counter & 0xF);
    }
    else
    {
        s += (m_data.back() & 0xF);
    }

    if (!m_sPGN.compare(gk_pgnTSC1)) // TSC1
    {
        return (((s >> 6) & 0x3) + (s >> 3) + s) & 0x7;
    }
    else if (!m_sPGN.compare(gk_pgnXBR)) // XBR_DMC
    {
        return ((s >> 4) + s) & 0xF;
    }
    return s & 0xFF;
}

void CanDataInfo::PrintInfo() const
{
    // std::cout << "Time stamp: " << m_stamp.tv_sec << "." << std::setw(9) << std::setfill('0') << m_stamp.tv_usec;
    // std::cout << ", Data: ";
    // for (uint8_t i = 0; i < m_data.size(); ++i)
    // {
    //     std::cout << Byte::Int2Hex(m_data[i]) << " ";
    // }
    // LOG_WARN() << " out of bound with can id: " << m_id
    //            << ", pgn: " << m_sPGN.c_str();
}

void CanDataInfo::Init()
{
    m_sPGN = Byte::PGNFromCanId(m_id);

    if (m_id < 65536) // not greater than 0xFFFF
    {
        m_pgnType = PGNType::CUSTOM;
        m_sEcuAddress = "";
    }
    else
    {
        uint8_t high = static_cast<uint8_t>((m_id >> 16) & 0xFF); //& max value 0xFF for length 8, so limited length is 8
        if ((high & 0xF0) == 0xF0)                                // Begin with 0xF
        {
            m_pgnType = PGNType::BROADCAST;
            uint8_t srcId = m_id & 0xFF;
            m_sEcuAddress = Byte::Int2Hex(srcId);
        }
        else
        {
            if (high == 0xEC) // 0xEC00
            {
                m_pgnType = PGNType::TPCM;
            }
            else if (high == 0xEB) // 0xEB00
            {
                m_pgnType = PGNType::TPDT;
            }
            else // other PGN not Begin with 0xF
            {
                m_pgnType = PGNType::P2P;
            }
            std::string canId = Byte::Int2Hex(m_id);
            m_sEcuAddress = canId.substr(canId.length() - 4);
        }
    }
}

END_NS_ZF_DRIVER_CANBUS