#ifndef ZF_FRAMEWORK_MESSAGE_BASE_H
#define ZF_FRAMEWORK_MESSAGE_BASE_H

#include <memory>
#include <string>
#include <cstring>

#include "zf_global/in/zf_framework_global.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

    class MsgBase
// : public std::enable_shared_from_this<MsgBase>
{
public:
    // using Base = std::enable_shared_from_this<MsgBase>;
    // using Ptr = std::shared_ptr<MsgBase>;
    MsgBase() : type_name_("") {}
    MsgBase(uint32_t iType) : m_msgType(iType), type_name_("") {}
    MsgBase(const std::string &msg, const uint64_t &timestamp)
        : data_(msg), m_timestamp(timestamp) {}
    MsgBase(const MsgBase &msg)
        : data_(msg.data_), m_msgType(msg.m_msgType), type_name_(msg.type_name_), m_timestamp(msg.m_timestamp) {}
    virtual ~MsgBase() {}

    virtual bool SerializeToArray(void *data, int size) const;
    bool SerializeToString(std::string *output) const;
    virtual bool ParseFromArray(const void *data, int size);
    bool ParseFromString(const std::string &msgstr);
    int ByteSize() const { return static_cast<int>(sizeof(uint64_t) + data_.size()); };

    const std::string &Data() const;
    void SetData(const std::string &msg);
    const uint64_t &TimeStamp() const;
    void SetTimeStamp(const uint64_t &timestamp);
    const std::string &type_name() { return type_name_; };
    void set_type_name(const std::string &type_name) { type_name_ = type_name; };
    uint32_t GetMsgType() const
    {
        return m_msgType;
    }
    // // Template specialization for RawMessage
    // static bool SerializeToArray(const MsgBase &message, void *data,
    //                              int size)
    // {
    //     return message.SerializeToArray(data, size);
    // }

    // static bool ParseFromArray(const void *data, int size, MsgBase *message)
    // {
    //     return message->ParseFromArray(data, size);
    // }

    // static int ByteSize(const MsgBase &message) { return message.ByteSize(); }
    // Ptr Shared();

private:
    // MsgBase(const MsgBase &) = delete;
    // MsgBase &operator=(const MsgBase &) = delete;

protected:
    std::string data_;
    uint64_t m_timestamp = 0;
    uint32_t m_msgType;
    std::string type_name_;
};

// inline MsgBase::Ptr MsgBase::Shared()
// {
//     return shared_from_this();
// }

inline void MsgBase::SetData(const std::string &msg) { data_ = msg; }

inline const std::string &MsgBase::Data() const { return data_; }

inline const uint64_t &MsgBase::TimeStamp() const { return m_timestamp; }

inline void MsgBase::SetTimeStamp(const uint64_t &timestamp)
{
    m_timestamp = timestamp;
}

inline bool MsgBase::ParseFromArray(const void *data, int size)
{
    if (data == nullptr || size <= 0)
    {
        return false;
    }
    void *ptr = const_cast<void *>(data);
    std::memcpy(reinterpret_cast<char *>(&m_timestamp), ptr, sizeof(uint64_t));
    ptr += sizeof(uint64_t);
    data_.assign(reinterpret_cast<const char *>(ptr), size - sizeof(uint64_t));

    return true;
}

inline bool MsgBase::ParseFromString(const std::string &msgstr)
{
    // todo : will use submsg type ywf
    // std::size_t pos = msgstr.rfind(data_split_pattern);
    // if (pos != std::string::npos) {
    //   std::size_t split_count = data_split_pattern.size();
    //   data_ = msgstr.substr(0, pos);
    //   type_name_ = msgstr.substr(pos + split_count);
    //   return true;
    // }
    data_ = msgstr;
    return true;
}

inline bool MsgBase::SerializeToArray(void *data, int size) const
{
    if (data == nullptr || size < ByteSize())
    {
        return false;
    }
    void *ptr = data;
    std::memcpy(ptr, reinterpret_cast<const char *>(&m_timestamp), sizeof(uint64_t));
    ptr += sizeof(uint64_t);
    memcpy(ptr, data_.data(), data_.size());

    return true;
}

inline bool MsgBase::SerializeToString(std::string *output) const
{
    if (!output)
    {
        return false;
    }
    // todo : will use submsg type ywf
    // *output = data_ + data_split_pattern + type_name_;
    *output = data_;
    return true;
}

template <class Msg>
class WrappedMsg
    : public MsgBase
{
public:
    using Base = MsgBase;
    using Ptr = std::shared_ptr<WrappedMsg>;

    typedef Msg MsgContent;

    WrappedMsg(UInt iType,
               const MsgContent &oContent);
    ~WrappedMsg(void);

    void SetData(const MsgContent &oContent);

    const MsgContent &GetData(void);

private:
    WrappedMsg(const WrappedMsg &) = delete;
    WrappedMsg &operator=(const WrappedMsg &) = delete;

    MsgContent m_oContent;
};

template <class Msg>
WrappedMsg<Msg>::WrappedMsg(UInt iType,
                            const MsgContent &oContent)
    : Base(iType), m_oContent(oContent)
{
}

template <class Msg>
WrappedMsg<Msg>::~WrappedMsg()
{
}

template <class Msg>
void WrappedMsg<Msg>::SetData(const MsgContent &oContent)
{
    m_oContent = oContent;
}

template <class Msg>
const Msg &WrappedMsg<Msg>::GetData()
{
    return m_oContent;
}

END_NS_ZF_FRAMEWORK

#endif // ZF_FRAMEWORK_MESSAGE_BASE_H