#include "zf_hal_can_driver/hal_can_id.h"
#include "zf_hal_can_driver/client/can_frame.h"

#include <linux/can.h> // for CAN typedef so I can static_assert it
#include <utility>

BEGIN_NS_ZF_DRIVER_CANBUS

constexpr uint32_t EXTENDED_MASK = CAN_EFF_FLAG;
constexpr uint32_t REMOTE_MASK = CAN_RTR_FLAG;
constexpr uint32_t ERROR_MASK = CAN_ERR_FLAG;
constexpr uint32_t EXTENDED_ID_MASK = CAN_EFF_MASK;
constexpr uint32_t STANDARD_ID_MASK = CAN_SFF_MASK;

uint32_t CanFrame::IdentifierID() const
{
    return (id & EXTENDED_MASK) == EXTENDED_MASK ? (id & EXTENDED_ID_MASK) : (id & STANDARD_ID_MASK);
}

timeval CanFrame::to_timeval(const std::chrono::nanoseconds timeout) noexcept
{
    const auto count = timeout.count();
    constexpr auto BILLION = 1'000'000'000LL;
    struct timeval c_timeout;
    c_timeout.tv_sec = static_cast<decltype(c_timeout.tv_sec)>(count / BILLION);
    c_timeout.tv_usec = static_cast<decltype(c_timeout.tv_usec)>((count % BILLION) / 1000LL);
    return c_timeout;
}

uint64_t CanFrame::from_timeval(const timeval tv) noexcept
{
    return static_cast<uint64_t>(tv.tv_sec) * 1e6 + tv.tv_usec;
}

uint32_t CanId::Identifier(const uint32_t &raw_id)
{
    return (raw_id & EXTENDED_MASK) == EXTENDED_MASK ? (raw_id & EXTENDED_ID_MASK) : (raw_id & STANDARD_ID_MASK);
}

uint32_t CanId::GenerateCanId(uint8_t priority, const std::string &pgn, uint8_t sourceId, uint8_t destId)
{
    uint32_t canId = 0;
    // Set priority bits (bits 28-26) in the CAN ID
    canId |= (priority & 0x07) << 26;
    // Convert PGN in HEX from string to uint32_t
    uint32_t pgnValue = std::stoul(pgn, nullptr, 16);
    // Set PGN bits (bits 25-8) in the CAN ID
    canId |= (pgnValue & 0x3FFFF) << 8;
    // Set source ID bits (bits 7-0) in the CAN ID
    canId |= sourceId;
    // Set destination ID bits (bits 15-8) in the CAN ID
    canId |= (destId & 0xFF) << 8;
    return canId;
}

uint32_t CanId::GenerateCanId(uint32_t priority, const uint32_t &pgn, const std::string &ecuAddr)
{
    uint32_t canId = 0;
    // // Set priority bits (bits 28-26) in the CAN ID
    // canId |= (priority & 0x07) << 26;
    canId |= priority << 24;

    // Set PGN bits (bits 25-8) in the CAN ID
    canId |= (pgn & 0x3FFFF) << 8;
    uint32_t sourceId = std::stoul(ecuAddr, nullptr, 16);
    uint8_t high = static_cast<uint8_t>((pgn >> 8) & 0xFF); //& max value 0xFF for length 8, so limited length is 8
    if ((high & 0xF0) == 0xF0)                              // Begin with 0xF
    {
        // Set source ID bits (bits 7-0) in the CAN ID
        canId |= (sourceId & 0xFF);
    }
    else
    {
        // Set source ID bits (bits 7-0) in the CAN ID
        canId |= sourceId;
    }
    return canId;
}

// For recv
CanId::CanId(const uint32_t raw_id) : m_idRaw{raw_id}
{
    Identifier();
    (void)InitFrameType();
}

// For send
CanId::CanId(const uint32_t id, FrameType type, bool is_extended) : m_idRaw{id}
{
    (void)SetIDType(is_extended); // Set extended bit
    (void)SetFrameType(type);
    Identifier();
    (void)Init(id);
}

uint32_t CanId::IdRaw() const noexcept
{
    return m_idRaw;
}

uint32_t CanId::Id() const noexcept
{
    return m_id;
}

bool CanId::IsExtended() const noexcept
{
    return (m_idRaw & EXTENDED_MASK) == EXTENDED_MASK;
}

CanId &CanId::SetIDType(bool is_extended) noexcept
{
    // lint -e{9126} NOLINT false positive: underlying type is unsigned long, and same as m_idRaw
    if (is_extended) // Extended: Sets bit 31 to 1
    {
        m_idRaw = m_idRaw | EXTENDED_MASK;
    }
    else // Standard: Sets bit 31 to 0
    {
        m_idRaw = m_idRaw & (~EXTENDED_MASK);
    }
    return *this;
}

CanId &CanId::InitFrameType()
{
    const bool is_error = (m_idRaw & ERROR_MASK) == ERROR_MASK;
    const bool is_remote = (m_idRaw & REMOTE_MASK) == REMOTE_MASK;
    if (is_error && is_remote)
    {
        // LOG_ERROR() << "m_idRaw is" << m_idRaw;
        // std::stringstream ss;
        // ss << std::uppercase << std::setfill('0') << std::setw(8) << std::hex << (m_idRaw | 0);
        // LOG_ERROR() << "m_idRawHex is" << ss.str().c_str();
        throw std::domain_error{"CanId has both bits 29 and 30 set! Inconsistent!"};
    }

    if (is_error)
    {
        m_type = FrameType::ERROR;
    }
    else if (is_remote)
    {
        m_type = FrameType::REMOTE;
    }
    else
    {
        m_type = FrameType::DATA;
    }
    return *this;
}

CanId &CanId::SetFrameType(const FrameType type)
{
    m_type = type;
    switch (type)
    {
    case FrameType::DATA: // Clears bits 29 and 30 (sets to 0)
        m_idRaw = m_idRaw & (~ERROR_MASK);
        m_idRaw = m_idRaw & (~REMOTE_MASK);
        break;
    case FrameType::ERROR: // Sets bit 29 to 1, and bit 30 to 0
        m_idRaw = m_idRaw & (~REMOTE_MASK);
        m_idRaw = m_idRaw | ERROR_MASK;
        break;
    case FrameType::REMOTE: // Sets bit 29 to 0, and bit 30 to 1
        m_idRaw = m_idRaw & (~ERROR_MASK);
        m_idRaw = m_idRaw | REMOTE_MASK;
        break;
    default:
        throw std::logic_error{"CanId: No such type"};
    }
    return *this;
}

FrameType CanId::GetFrameType() const
{
    return m_type;
}

CanId &CanId::Init(const uint32_t id)
{
    // Can specification: http://esd.cs.ucr.edu/webres/can20.pdf
    // says "The 7 most significant bits cannot all be recessive (value of 1)", pg 11
    constexpr auto MAX_EXTENDED = 0x1FBF'FFFFU;
    constexpr auto MAX_STANDARD = 0x07EFU;
    static_assert(MAX_EXTENDED <= EXTENDED_ID_MASK, "Max extended id value is wrong");
    static_assert(MAX_STANDARD <= STANDARD_ID_MASK, "Max extended id value is wrong");
    const auto max_id = IsExtended() ? MAX_EXTENDED : MAX_STANDARD;
    if (max_id < id)
    {
        throw std::domain_error{"CanId would be truncated!"};
    }
    // Clear and set
    // lint -e{9126} NOLINT false positive: underlying type is unsigned long, and same as m_idRaw
    m_idRaw = m_idRaw & (~EXTENDED_ID_MASK); // clear ALL ID bits, not just standard bits
    m_idRaw = m_idRaw | id;

    return *this;
}

void CanId::Identifier() noexcept
{
    const auto mask = IsExtended() ? EXTENDED_ID_MASK : STANDARD_ID_MASK;
    m_id = m_idRaw & mask;
}

END_NS_ZF_DRIVER_CANBUS