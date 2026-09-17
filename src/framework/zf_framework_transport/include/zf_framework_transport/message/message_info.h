/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef CYBER_TRANSPORT_MESSAGE_INFO_H_
#define CYBER_TRANSPORT_MESSAGE_INFO_H_

#include <cstddef>
#include <cstdint>
#include <string>

#include "zf_framework_transport/message/identity.h"
#include "zf_global/util/string_util.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework
#include <string>

    class RoleAttributes
{
public:
  RoleAttributes() = default;
  // Copy constructor
  RoleAttributes(const RoleAttributes &other)
      : hostName(other.hostName),
        hostIp(other.hostIp),
        channelName(other.channelName),
        channelId(other.channelId),
        id(other.id)
  {
  }

  // Copy assignment operator
  RoleAttributes &operator=(const RoleAttributes &other)
  {
    if (this != &other)
    {
      hostName = other.hostName;
      hostIp = other.hostIp;
      channelName = other.channelName;
      channelId = other.channelId;
      id = other.id;
    }
    return *this;
  }
  const std::string &GetHostName() const;
  void SetHostName(const std::string &name);
  const std::string &GetHostIp() const;
  void SetHostIp(const std::string &ip);
  const std::string &GetChannelName() const;
  void SetChannelName(const std::string &name);
  uint64_t GetChannelId() const;
  void SetChannelId(uint64_t id);
  const uint32_t &GetId() const;
  void SetId(const uint32_t &idValue);
  bool IsEmpty() const;

private:
  std::string hostName;
  std::string hostIp;
  std::string channelName = "";
  uint64_t channelId;
  uint32_t id;
  bool empty = true;
};

class MessageInfo
{
public:
  MessageInfo();
  MessageInfo(const Identity &sender_id, uint64_t seq_num);
  MessageInfo(const Identity &sender_id, uint64_t seq_num,
              const Identity &spare_id);
  MessageInfo(const MessageInfo &another);
  virtual ~MessageInfo();

  MessageInfo &operator=(const MessageInfo &another);
  bool operator==(const MessageInfo &another) const;
  bool operator!=(const MessageInfo &another) const;

  bool SerializeTo(std::string *dst) const;
  bool SerializeTo(char *dst, std::size_t len) const;
  bool DeserializeFrom(const std::string &src);
  bool DeserializeFrom(const char *src, std::size_t len);

  // Getter and Setter
  const Identity &sender_id() const { return sender_id_; }
  void set_sender_id(const Identity &sender_id) { sender_id_ = sender_id; }

  uint64_t channel_id() const { return channel_id_; }
  void set_channel_id(uint64_t channel_id) { channel_id_ = channel_id; }

  uint64_t seq_num() const { return seq_num_; }
  void set_seq_num(uint64_t seq_num) { seq_num_ = seq_num; }

  const Identity &spare_id() const { return spare_id_; }
  void set_spare_id(const Identity &spare_id) { spare_id_ = spare_id; }

  static const std::size_t kSize;

private:
  Identity sender_id_;
  uint64_t channel_id_ = 0;
  uint64_t seq_num_ = 0;
  Identity spare_id_;
};

END_NS_ZF_FRAMEWORK

#endif // CYBER_TRANSPORT_MESSAGE_INFO_H_
