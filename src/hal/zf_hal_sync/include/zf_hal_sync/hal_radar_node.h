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
 * @brief Defines the hal datas sync ros node.
 * @return
 */

#ifndef ZF_HAL_RADAR_PACK_NODE_H_
#define ZF_HAL_RADAR_PACK_NODE_H_

#include <mutex>
// #include <deque>
#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/can_datas.hpp"
#include "can_msgs/msg/can_msg_data.hpp"
#include "can_msgs/msg/can_sig_data.hpp"
#include "can_msgs/msg/radar_cam_datas.hpp"
#include "can_msgs/msg/frame.hpp"

#include "zf_hal_sync/hal_radar_pack_policy.h"
#include "zf_global/in/hal_sync_global.h"

// /**
//  * @namespace NS_ZF::driver::synchronization
//  */
BEGIN_NS_ZF_HAL_SYNC

class HalRadarPackNode : public rclcpp::Node
{
public:
	using RadarPackPolicyPtr = std::shared_ptr<RadarPackPolicy>;
	using RadarPackMap = std::unordered_map<uint32_t, RadarPackPolicyPtr>;
	using SubPackMap = std::unordered_map<uint32_t, rclcpp::Subscription<can_msgs::msg::CanMsgData>::SharedPtr>;
	using PubPackMap = std::unordered_map<uint32_t, rclcpp::Publisher<can_msgs::msg::CanDatas>::SharedPtr>;
	explicit HalRadarPackNode(const rclcpp::NodeOptions &, std::string name = "hal_radar_pack_node");
	~HalRadarPackNode();

private:
	void RadarDataCallback(const can_msgs::msg::CanMsgData::ConstSharedPtr msg, ERadarType type);

	std::string LogString(const can_msgs::msg::CanMsgData &msgData);
	void LogStrings(const can_msgs::msg::CanDatas &msgDatas);

private:
	// std::string LogString(const can_msgs::msg::CanMsgData &msg);
	bool m_bEnableDebug;
	RadarPackMap m_mapRadarPackers;
	SubPackMap m_subCanMsgData;
	PubPackMap m_pubRadarData;
	// rclcpp::TimerBase::SharedPtr m_timerCan1;
	// rclcpp::TimerBase::SharedPtr m_timerPcan0;
};

END_NS_ZF_HAL_SYNC

#endif /* ZF_HAL_RADAR_PACK_NODE_H_ */