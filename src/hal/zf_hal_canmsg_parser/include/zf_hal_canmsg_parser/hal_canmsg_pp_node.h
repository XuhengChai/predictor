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
 * @brief Defines the can parser pack ros node.
 * @return
 */

#ifndef ZF_HAL_CANMSG_PARSER_PACK_NODE_H_
#define ZF_HAL_CANMSG_PARSER_PACK_NODE_H_

#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/can_datas.hpp"
#include "can_msgs/msg/can_msg_data.hpp"
#include "can_msgs/msg/can_sig_data.hpp"
#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/string.hpp"

#include "zf_hal_canmsg_parser/hal_canmsg_parser.h"

// /**
//  * @namespace NS_ZF::driver::canbus
//  */
BEGIN_NS_ZF_DRIVER_CANBUS

class CanMsgParserPackNode : public rclcpp::Node
{
public:
	using PGN_CANDatas_map = std::unordered_map<std::string, can_msgs::msg::CanDatas::SharedPtr>; //  From which PGN came this message, Can datas
	explicit CanMsgParserPackNode(const rclcpp::NodeOptions &, std::string name = "can_msg_parser_pack");
	~CanMsgParserPackNode();

private:
	enum ETransNode : uint32_t
	{
		TRANS2_EGO = 0,
		TRANS2_UI = 1,
	};
	void FillInCanData(const std::string &name);
	int Index(const uint32_t &msg_id);
	void ParseRawCanDataCallback(const can_msgs::msg::Frame::ConstSharedPtr msg);
	void VehicleRawCanDataCallback(const can_msgs::msg::Frame::ConstSharedPtr msg);
	void PackMsgDataCallback(const can_msgs::msg::CanMsgData::ConstSharedPtr msg);
	void PackSigDataCallback(const can_msgs::msg::CanSigData::ConstSharedPtr msg);
	NS_ZF::ErrorCode Raw2MsgData(const can_msgs::msg::Frame::ConstSharedPtr m, can_msgs::msg::CanMsgData &msgData, CanMsgParser::SignalPPMap &);
	NS_ZF::ErrorCode Msg2RawData(const can_msgs::msg::CanMsgData::ConstSharedPtr msgData, can_msgs::msg::Frame &raw);
	NS_ZF::ErrorCode Sig2RawData(const can_msgs::msg::CanSigData::ConstSharedPtr sigData, can_msgs::msg::Frame &raw);

private:
	void InitTimerTransmit();
	bool PubAEBS2_27ForAC1000T();
	bool PubTSC1();
	bool PubVehicleCanMsg2Ego();
	bool PubXBR();
	bool ModifyDataTo5G4T(can_msgs::msg::Frame &data, const std::string &pgn);
	std::string LogString(const can_msgs::msg::CanMsgData &msg);
	void InitSet();
	void InitVehicleVars();
	std::string m_strInterface;
	std::string m_strVehicleInterface;
	int m_iCanDescription;
	StringVector m_vDbcFilesPath;
	StringVector m_vDbcName;
	int m_iLogCnt;
	bool m_bEnableDebug;
	bool m_isVehicleCan = {};
	PGN_CANDatas_map m_pCanDatas;
	rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr m_subRawCanData;
	rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr m_subVehicleCanData;
	rclcpp::Publisher<can_msgs::msg::CanMsgData>::SharedPtr m_pubMsgData;

	rclcpp::Subscription<can_msgs::msg::CanMsgData>::SharedPtr m_subMsgData;
	rclcpp::Subscription<can_msgs::msg::CanSigData>::SharedPtr m_subSigData;
	rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr m_pubRawCanData;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pubStwInfo;

	// std::mutex m_mtx;
	std::unordered_map<int, std::set<std::string>> m_mapPgnSet;
	std::set<uint32_t> m_setTransferIds;
	std::unordered_map<uint32_t, uint32_t> m_mapTransferIds; // val: 0, trans to
	rclcpp::TimerBase::SharedPtr m_timerOWW;				 //
	can_msgs::msg::Frame m_canDataOWW;
	rclcpp::TimerBase::SharedPtr m_timerVDHR_TDEE; // 1000ms
	can_msgs::msg::Frame m_canDataVDHR;
	// rclcpp::TimerBase::SharedPtr m_timerTDEE; // 10ms
	can_msgs::msg::Frame m_canDataTD_EE;
	rclcpp::TimerBase::SharedPtr m_timerTSC1; // 10ms
	can_msgs::msg::CanMsgData m_canMsgDataTSC1;
	can_msgs::msg::Frame m_canDataTSC1;
	std::mutex m_lockTSC1;
	rclcpp::TimerBase::SharedPtr m_timerXBR; // 20ms
	can_msgs::msg::CanMsgData m_canMsgDataXBR;
	can_msgs::msg::Frame m_canDataXBR;
	std::mutex m_lockXBR;

	rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr m_pubNodeStatus;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pubStrRadarStatus;
	std_msgs::msg::String m_msgStrRadarStatus;
	int32_t m_iB0Status;
	int32_t m_iC0Status;
	rclcpp::TimerBase::SharedPtr m_timerNodeSataus;
	std_msgs::msg::UInt32 m_statusMsg;
	std::vector<uint8_t> m_vCntMsgTimeout;
	std::unordered_map<uint32_t, std::vector<uint32_t>> m_indexMsgTimeout;
	uint32_t m_iCnt10ms = {};
	std::unordered_map<uint32_t, std::vector<SignalAlias>> m_mapSigsVehicle;
	CanMsgParser::SignalPPMap m_mapSigs2Ego;
	can_msgs::msg::CanMsgData m_canMsgData2Ego;
	std::mutex m_lockSigs2Ego;

private:											 // 5g4t
	rclcpp::TimerBase::SharedPtr m_timerTrailer5G4T; // TrailerPresentState
	can_msgs::msg::Frame m_canDataTrailer5G4T;

private:											  // AC100T
	rclcpp::TimerBase::SharedPtr m_timerERC1_AC1000T; // TrailerPresentState
	can_msgs::msg::Frame m_canDataERC1_29_AC1000T;
	rclcpp::TimerBase::SharedPtr m_timerCCSS_AC1000T; //
	can_msgs::msg::Frame m_canDataCCSS_AC1000T;
	rclcpp::TimerBase::SharedPtr m_timerFLIC_E8_27_AC1000T; //
	can_msgs::msg::Frame m_canDataFLIC_E8_27_AC1000T;
	can_msgs::msg::CanMsgData m_canMsgDataAEBS2_27;
	can_msgs::msg::Frame m_canDataAEBS2_27_AC1000T;
};

END_NS_ZF_DRIVER_CANBUS

#endif /* ZF_HAL_CANMSG_PARSER_PACK_NODE_H_ */