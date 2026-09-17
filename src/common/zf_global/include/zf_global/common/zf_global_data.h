/******************************************************************************
 * Copyright 2023 ZF. All Rights Reserved.
 * Author:
 *        xuheng.chai@zf.com
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
/**
 * @file
 * @brief Global data defines.
 * @run
 */

#ifndef ZF_GLOBAL_COMMON_DATA_H
#define ZF_GLOBAL_COMMON_DATA_H

#include <unordered_map>

#include "zf_global/common/zf_global_macros.h"
#include "zf_global/util/string_util.h"

BEGIN_NS_ZF

class GlobalData
{
public:
  using MapContainer = std::unordered_map<uint64_t, std::string>;
  static GlobalData &Instance()
  {
    static GlobalData ins;
    return ins;
  }
  ~GlobalData(){};

  uint64_t GenerateHashId(const std::string &name)
  {
    return NS_ZF_STRING_UTIL::Hash(name);
  }
  const std::string &HostIp() const { return host_ip_; };

  // uint64_t RegisterNode(const std::string& node_name);
  // std::string GetNodeById(uint64_t id);

  uint64_t RegisterChannel(const std::string &channel)
  {
    return Register(channel, channel_id_map_);
  };
  std::string GetChannelById(uint64_t id)
  {
    return GetId(id, channel_id_map_);
  };

  // uint64_t RegisterService(const std::string& service);
  // std::string GetServiceById(uint64_t id);

  // uint64_t RegisterTaskName(const std::string& task_name);
  // std::string GetTaskNameById(uint64_t id);

private:
  uint64_t Register(const std::string &channel, MapContainer &map)
  {
    auto id = NS_ZF_STRING_UTIL::Hash(channel);
    while (map.count(id))
    {
      std::string name = map[id];
      if (channel == name)
      {
        break;
      }
      ++id;
      LOG_WARN() << "Channel name hash collision: " << channel << " <=> " << name;
    }
    map[id] = channel;
    return id;
  };
  std::string GetId(const uint64_t &id, const MapContainer &map)
  {
    if (map.count(id))
    {
      return map.at(id);
    }
    return "";
  };
  // host info
  std::string host_ip_;
  std::string host_name_;

  // // process info
  // int process_id_;
  // std::string process_group_;

  // int component_nums_ = 0;

  // sched policy info
  // std::string sched_name_ = "CYBER_DEFAULT";
  // std::unordered_map<uint64_t, std::string> node_id_map_;
  std::unordered_map<uint64_t, std::string> channel_id_map_;
  // std::unordered_map<uint64_t, std::string> service_id_map_;
  // std::unordered_map<uint64_t, std::string> task_id_map_;
  // DECLARE_SINGLETON(GlobalData)
  GlobalData()
  {
    host_ip_ = "127.0.0.1";
  };
  DISALLOW_COPY_AND_ASSIGN(GlobalData)
};

END_NS_ZF

#endif // !ZF_GLOBAL_COMMON_MACROS_H