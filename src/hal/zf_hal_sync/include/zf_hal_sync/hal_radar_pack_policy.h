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
 * @brief Defines the hal radar data pack policy.
 * @return
 */

#ifndef ZF_HAL_RADAR_PACK_POLICY_H_
#define ZF_HAL_RADAR_PACK_POLICY_H_

#include <mutex>
// #include <deque>
#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/can_datas.hpp"
#include "can_msgs/msg/can_msg_data.hpp"
#include "can_msgs/msg/can_sig_data.hpp"
#include "can_msgs/msg/radar_cam_datas.hpp"
#include "can_msgs/msg/frame.hpp"

#include "zf_global/in/hal_sync_global.h"
#include "zf_global/util/string_util.h"
#include "zf_global/util/logger.h"

// /**
//  * @namespace NS_ZF::driver::synchronization
//  */
BEGIN_NS_ZF_HAL_SYNC

using RadarData = can_msgs::msg::CanDatas;
using ObjDataPtr = can_msgs::msg::CanDatas::SharedPtr;
using LanesDataPtr = ObjDataPtr;

struct PackedRadarData
{
	// uint32_t objNumTotal;
	uint32_t radarType;
	// uint32_t objNumCurB;
	// uint32_t objNumCurC;
	// uint32_t objNumCurD;
	// TimeSync timestamp;
	ObjDataPtr ptrObjData;
	// LanesDataPtr ptrLaneData;
};

static const char gk_radarNovaObjNum[] = "Obj_NofObjects";		   // 60A
static const char gk_radar5G4TLeftB00A[] = "SRR_Left_B0_Obj_0A";   // 2566897328
static const char gk_radar5G4TRightC00A[] = "SRR_Right_C0_Obj_0A"; // 2566897328
// ORDER C0 C1 B1 B0
static const char gk_radar5G4TLeftB0[] = "SRR_Left_B0_Obj";
static const char gk_radar5G4TLeftB1[] = "SRR_Left_B1_Obj";
static const char gk_radar5G4TRightC0[] = "SRR_Right_C0_Obj";
static const char gk_radar5G4TRightC1[] = "SRR_Right_C1_Obj";
// static const char gk_radar5G4TRightC0Status[] = "Right_C0_Status";

LOGGER::FileLogger LOG_FILE_PACKER_ERROR;

class RadarPackPolicy
{
public:
	using RadarDataMap = std::unordered_map<uint32_t, PackedRadarData>; // status can id 60a, 61a
	explicit RadarPackPolicy(std::string ns, uint32_t radarType) : m_iRadarType(radarType), m_strNs(ns) {};
	virtual ~RadarPackPolicy() {};
	// return -1; no publish; else: publish returned value;
	virtual int PackedRadarDataCallback(const can_msgs::msg::CanMsgData::ConstSharedPtr msg) = 0;
	std::string GetPrefix() { return m_strNs; };
	uint32_t GetRardarType() { return m_iRadarType; }
	ObjDataPtr GetObjData(uint32_t id)
	{
		return m_mapRadarDatas.at(id).ptrObjData;
	}
	double GetValueBySigName(const can_msgs::msg::CanMsgData::ConstSharedPtr msg, const std::string &sigName)
	{
		for (auto &sig : msg->sig_datas)
		{
			if (!sigName.compare(sig.sig_name))
			{
				return sig.sig_data;
			}
		}
	}

	bool CheckRadarDataValid(const uint32_t &id) const
	{
		auto ite = m_mapRadarDatas.find(id);
		if (ite == m_mapRadarDatas.end())
		{
			return false;
		}
		return true;
	}

	virtual bool RemoveRadarData(const uint32_t &id)
	{
		auto ite = m_mapRadarDatas.find(id);
		if (ite == m_mapRadarDatas.end())
		{
			return false;
		}
		m_mapRadarDatas.erase(ite);
		return true;
	}

	virtual bool ResetRadarData(const uint32_t &id)
	{
		RemoveRadarData(id);
	}

	std::string LogString(const can_msgs::msg::CanMsgData &msgData)
	{
		std::stringstream output_stream("");
		std_msgs::msg::Header head;
		output_stream << "[" << msgData.header.stamp.sec << "." << std::setw(9) << std::setfill('0') << msgData.header.stamp.nanosec
					  << "], "
					  << msgData.msg_name << ", " << msgData.msg_pgn;
		return output_stream.str();
	}

	void LogStrings(const can_msgs::msg::CanDatas &msgDatas, bool end = true)
	{
		for (auto &data : msgDatas.msg_datas)
		{
			LOG_FILE_PACKER_ERROR() << LogString(data);
		}
		if (end)
		{
			LOG_FILE_PACKER_ERROR() << "----error end";
		}
	}

protected:
	uint32_t m_iRadarType;
	std::string m_strNs;
	RadarDataMap m_mapRadarDatas;
};

class RadarIPMPack : public RadarPackPolicy
{
public:
	RadarIPMPack(std::string ns, uint32_t radarType = ERadarType::RADAR_IPM) : RadarPackPolicy(ns, radarType) {};
	virtual ~RadarIPMPack() {};
	virtual int PackedRadarDataCallback(const can_msgs::msg::CanMsgData::ConstSharedPtr msg) override
	{
		// status
		uint32_t tId = 0x67F; // begin with 67F: FC_Video_Data_General_A
		bool bhasId = CheckRadarDataValid(tId);
		if ((msg->msg_id & 0xFFF) == tId) // FC_Video_Data_General_A
		{
			if (bhasId)
			{
				LOG_ERROR() << "IPM obj msg missing, objNum: " << m_mapRadarDatas[tId].ptrObjData->data_size
							<< ", cur objNum: " << m_mapRadarDatas[tId].ptrObjData->msg_datas.size();
				// << ", laneNum: " << m_mapRadarDatas[tId].ptrLaneData->data_size
				// << ", cur laneNum: " << m_mapRadarDatas[tId].ptrLaneData->msg_datas.size();
				LOG_FILE_PACKER_ERROR() << "IPM obj msg missing, objNum: " << m_mapRadarDatas[tId].ptrObjData->data_size
							<< ", cur objNum: " << m_mapRadarDatas[tId].ptrObjData->msg_datas.size();
				// << ", laneNum: " << m_mapRadarDatas[tId].ptrLaneData->data_size
				// << ", cur laneNum: " << m_mapRadarDatas[tId].ptrLaneData->msg_datas.size();
				LogStrings(*m_mapRadarDatas[tId].ptrObjData);
			}
			ObjDataPtr objData = std::make_shared<RadarData>();
			objData->header = msg->header;
			objData->data_size = 61; // 3 + 1 + 12 * 4 = 52
			objData->msg_datas.push_back(*msg);

			// LanesDataPtr laneData = std::make_shared<RadarData>();
			// laneData->header = msg->header;
			// laneData->data_size = 9; // 4*2 + 1
			// radarData->data_size = this->GetValueBySigName(msg, "FC_Gen_MessageCounter_A");
			PackedRadarData tData;
			tData.radarType = m_iRadarType;
			// tData.objNumTotal = 61; // 3 + 1 + 12 * 4 + 4*2 + 1 -- ABC +Header + 12*4 obj + line + Line_Header1714
			// tData.objNumCurB = 0;
			// tData.objNumCurC = 0;
			// tData.objNumCurD = 0;
			tData.ptrObjData = objData;
			// tData.ptrLaneData = laneData;
			m_mapRadarDatas[tId] = tData;
			return -1;
		}
		else
		{
			if (!bhasId)
			{
				// LOG_INFO() << "Waiting for IPM Obj Status..." << LogString(*msg);
				return -1;
			}
		}
		// if ((msg->msg_id & 0xFFF) >= 0x06B2 &&
		// 	(msg->msg_id & 0xFFF) <= 0x06BA) // 1714 to 1722
		// {
		// 	m_mapRadarDatas[tId].ptrLaneData->msg_datas.push_back(*msg);
		// }
		// else if ((msg->msg_id & 0xFFF) >= 0x0680 &&
		// 		 (msg->msg_id & 0xFFF) <= 0x06BF) // 1664 to 1727. TODO: if there are 1723 to 1726
		// {
		// 	m_mapRadarDatas[tId].ptrObjData->msg_datas.push_back(*msg);
		// }
		// if (m_mapRadarDatas[tId].ptrObjData->msg_datas.size() == m_mapRadarDatas[tId].ptrObjData->data_size &&
		// 	m_mapRadarDatas[tId].ptrLaneData->msg_datas.size() == m_mapRadarDatas[tId].ptrLaneData->data_size)
		// {
		// 	return tId;
		// }
		if ((msg->msg_id & 0xFFF) >= 0x0680 &&
			(msg->msg_id & 0xFFF) <= 0x06BF) // 1664 to 1727. TODO: if there are 1723 to 1726
		{
			m_mapRadarDatas[tId].ptrObjData->msg_datas.push_back(*msg);
		}
		if (m_mapRadarDatas[tId].ptrObjData->msg_datas.size() == m_mapRadarDatas[tId].ptrObjData->data_size)
		{
			return tId;
		}
		return -1;
	}
};

class Radar5G4TPack : public RadarPackPolicy
{
private:
	uint32_t m_id;
	bool m_bHasB0;
	bool m_bHasC0;
	double m_dB0Time[2]; // begin last time
	double m_dC0Time[2];
	std::set<std::string> m_mapMsgNames;
	void InsertData(uint32_t index, uint32_t size)
	{
		ObjDataPtr objData = std::make_shared<RadarData>();
		// objData->header = msg->header;
		objData->data_size = size;
		// objData->msg_datas.push_back(*msg);
		PackedRadarData tData;
		tData.radarType = m_iRadarType;
		tData.ptrObjData = objData;
		m_mapRadarDatas[index] = tData;
	}

public:
	Radar5G4TPack(std::string ns, uint32_t radarType = ERadarType::RADAR_5G4T) : RadarPackPolicy(ns, radarType)
	{
		LOG_FILE_PACKER_ERROR.SetFileName("build_at_" __DATE__ "_packer_error_5g4t.log");
		m_id = 0xB0C0;
		m_bHasB0 = false;
		m_bHasC0 = false;
	};
	virtual ~Radar5G4TPack() {};
	virtual bool ResetRadarData(const uint32_t &id) override
	{
		RemoveRadarData(id);
		m_bHasB0 = false;
		m_bHasC0 = false;
		m_mapMsgNames.clear();
	}
	virtual int PackedRadarDataCallback(const can_msgs::msg::CanMsgData::ConstSharedPtr msg) override
	{
		// status
		// LOG_DEBUG() << "PackedRadarDataCallback----" << msg->msg_name
		// 			<< "[" << msg->header.stamp.sec << "." << std::setw(9)
		// 			<< std::setfill('0') << msg->header.stamp.nanosec << "], ";
		bool bhasId = CheckRadarDataValid(m_id);
		bool bIsB0 = !msg->msg_name.compare(gk_radar5G4TLeftB00A);
		bool bIsC0 = !msg->msg_name.compare(gk_radar5G4TRightC00A);
		// LOG_DEBUG() << bIsB0 << bIsC0  << m_bHasB0 << m_bHasC0<< "PackedRadarDataCallback----";
		if (bIsB0 || bIsC0) //
		{
			if (m_bHasB0 && m_bHasC0)
			{
				LOG_ERROR() << "5G4T obj msg missing, objNum: " << m_mapRadarDatas[m_id].ptrObjData->data_size
							<< ", cur objNum: " << m_mapRadarDatas[m_id].ptrObjData->msg_datas.size();
				LOG_FILE_PACKER_ERROR() << "obj missing, objNum: " << m_mapRadarDatas[m_id].ptrObjData->data_size
										<< ", cur objNum: " << m_mapRadarDatas[m_id].ptrObjData->msg_datas.size();
				// << ", laneNum: " << m_mapRadarDatas[tId].ptrLaneData->data_size
				// << ", cur laneNum: " << m_mapRadarDatas[tId].ptrLaneData->msg_datas.size();
				LogStrings(*m_mapRadarDatas[m_id].ptrObjData);
				ResetRadarData(m_id);
				bhasId = false;
			}
			if (bIsB0)
			{
				m_dB0Time[0] = rclcpp::Time(msg->header.stamp).seconds();
				m_dB0Time[1] = m_dB0Time[0];
				if ((m_bHasB0 == bIsB0) || (m_bHasC0 && (abs(m_dB0Time[0] - m_dC0Time[1]) > 0.03)))
				{
					LOG_ERROR() << "5G4T 2B or obj msg big timeB, objNum: " << m_mapRadarDatas[m_id].ptrObjData->msg_datas.size() << ", " << std::to_string(m_dB0Time[0])
								<< ", " << std::to_string(m_dC0Time[1]);
					LOG_FILE_PACKER_ERROR() << "5G4T 2B or obj msg big timeB, objNum: " << m_mapRadarDatas[m_id].ptrObjData->msg_datas.size() << ", " << std::to_string(m_dB0Time[0])
											<< ", " << std::to_string(m_dC0Time[1]);
					LogStrings(*m_mapRadarDatas[m_id].ptrObjData);
					ResetRadarData(m_id);
					bhasId = false;
				}
				m_bHasB0 = bIsB0;
			}
			else if (bIsC0)
			{
				m_dC0Time[0] = rclcpp::Time(msg->header.stamp).seconds();
				m_dC0Time[1] = m_dC0Time[0];
				if ((m_bHasC0 == bIsC0) || (m_bHasB0 && (abs(m_dB0Time[1] - m_dC0Time[0]) > 0.03)))
				{
					LOG_ERROR() << "5G4T 2C or obj msg big timeC, objNum: " << m_mapRadarDatas[m_id].ptrObjData->msg_datas.size() << ", " << std::to_string(m_dB0Time[1])
								<< ", " << std::to_string(m_dC0Time[0]);
					LOG_FILE_PACKER_ERROR() << "5G4T 2C or obj msg big timeC, objNum: " << m_mapRadarDatas[m_id].ptrObjData->msg_datas.size() << ", " << std::to_string(m_dB0Time[1])
											<< ", " << std::to_string(m_dC0Time[0]);
					LogStrings(*m_mapRadarDatas[m_id].ptrObjData);
					ResetRadarData(m_id);
					bhasId = false;
				}
				m_bHasC0 = bIsC0;
			}
			m_mapMsgNames.insert(msg->msg_name);
			if (bhasId)
			{
				m_mapRadarDatas[m_id].ptrObjData->msg_datas.push_back(*msg);
			}
			else
			{
				ObjDataPtr objData = std::make_shared<RadarData>();
				objData->header = msg->header;
				objData->data_size = 42; // objs + status
				objData->msg_datas.push_back(*msg);
				PackedRadarData tData;
				tData.radarType = m_iRadarType;
				tData.ptrObjData = objData;
				m_mapRadarDatas[m_id] = tData;
			}
			// LOG_DEBUG() << bIsB0 << bIsC0  << m_bHasB0 << m_bHasC0 << "bIsB0 || bIsC0----" << bhasId << "----" << m_mapMsgNames.size();
			return -1;
		}
		else
		{
			if (!bhasId)
			{
				// LOG_INFO() << "Waiting for Obj Status...";
				return -1;
			}
		}
		if (m_bHasB0)
		{
			if (
				NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TLeftB0, msg->msg_name) ||
				NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TS1B0, msg->msg_name))
			{
				m_mapMsgNames.insert(msg->msg_name);
				m_mapRadarDatas[m_id].ptrObjData->msg_datas.push_back(*msg);
				m_dB0Time[1] = rclcpp::Time(msg->header.stamp).seconds();
			}
		}
		if (m_bHasC0)
		{
			if (
				// NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TLeftB0, msg->msg_name) ||
				// NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TS1B0, msg->msg_name) ||
				// NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TLeftB1, msg->msg_name) ||
				NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TRightC0, msg->msg_name) ||
				NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TS5C0, msg->msg_name)
				// NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TRightC1, msg->msg_name)
			)
			{
				m_mapMsgNames.insert(msg->msg_name);
				m_mapRadarDatas[m_id].ptrObjData->msg_datas.push_back(*msg);
				m_dC0Time[1] = rclcpp::Time(msg->header.stamp).seconds();
				// LOG_WARN() << "IsSubStr " << msg->msg_name << "----" << m_mapMsgNames.size() << "---" << m_mapRadarDatas[m_id].ptrObjData->msg_datas.size();
			}
		}
		if (m_mapRadarDatas[m_id].ptrObjData->msg_datas.size() == m_mapRadarDatas[m_id].ptrObjData->data_size)
		{
			if (m_mapMsgNames.size() != m_mapRadarDatas[m_id].ptrObjData->data_size)
			{
				LOG_ERROR() << "5G4T repeated obj msg, objNum: " << m_mapRadarDatas[m_id].ptrObjData->data_size
							<< ", cur objNum: " << m_mapMsgNames.size();
				LOG_FILE_PACKER_ERROR() << "5G4T repeated obj msg, objNum: " << m_mapRadarDatas[m_id].ptrObjData->data_size
										<< ", cur objNum: " << m_mapMsgNames.size();
				// << ", laneNum: " << m_mapRadarDatas[tId].ptrLaneData->data_size
				// << ", cur laneNum: " << m_mapRadarDatas[tId].ptrLaneData->msg_datas.size();
				LogStrings(*m_mapRadarDatas[m_id].ptrObjData);
				ResetRadarData(m_id);
				return -1;
			}
			// LOG_INFO() << "SIZE DONE " << m_mapRadarDatas[m_id].ptrObjData->data_size << "----" << m_mapMsgNames.size() << "---" << m_mapRadarDatas[m_id].ptrObjData->msg_datas.size();
			return m_id;
		}
		return -1;
	}
	// virtual int PackedRadarDataCallback(const can_msgs::msg::CanMsgData::ConstSharedPtr msg) override
	// {bhasIdC0
	// 	uint32_t tId = 0x18FFBEB0; //
	// 	bool bhasId = CheckRadarDataValid(tId);
	// 	bool bhasIdB0 = CheckRadarDataValid(m_idB0);
	// 	bool bhasIdC0 = CheckRadarDataValid(m_idC0);
	// 	// LOG_INFO() << bhasIdB0 << "bhasIdB0 "  << msg->msg_name;
	// 	if (!msg->msg_name.compare(gk_radar5G4TRightC00A)) //
	// 	{
	// 		// LOG_INFO() << "--------------gk_radar5G4TRightC00A ";
	// 		if (bhasIdC0)
	// 		{
	// 			LOG_ERROR() << "obj C0 missing, objNum: " << m_mapRadarDatas[m_idC0].ptrObjData->data_size
	// 						<< ", cur objNum: " << m_mapRadarDatas[m_idC0].ptrObjData->msg_datas.size();
	// 			LOG_FILE_PACKER_ERROR() << "obj C0 missing, objNum: " << m_mapRadarDatas[m_idC0].ptrObjData->data_size
	// 									<< ", cur objNum: " << m_mapRadarDatas[m_idC0].ptrObjData->msg_datas.size();
	// 			// << ", laneNum: " << m_mapRadarDatas[tId].ptrLaneData->data_size
	// 			// << ", cur laneNum: " << m_mapRadarDatas[tId].ptrLaneData->msg_datas.size();
	// 			if (bhasIdB0)
	// 			{
	// 				LogStrings(*m_mapRadarDatas[m_idB0].ptrObjData, false);
	// 			}
	// 			LogStrings(*m_mapRadarDatas[m_idC0].ptrObjData);
	// 			RemoveRadarData(m_idB0);
	// 			RemoveRadarData(tId);
	// 		}
	// 		InsertData(m_idC0, 20);
	// 		m_mapRadarDatas[m_idC0].ptrObjData->header = msg->header;
	// 		m_mapRadarDatas[m_idC0].ptrObjData->msg_datas.push_back(*msg);
	// 		return -1;
	// 	}
	// 	else if (!msg->msg_name.compare(gk_radar5G4TLeftB00A)) //
	// 	{
	// 		// LOG_INFO() << "--------bhasIdB0 ";
	// 		// if (!bhasIdC0)
	// 		// {
	// 		// 	return -1;// B0 should between two C0; C0 first
	// 		// }
	// 		if (bhasIdB0)
	// 		{
	// 			LOG_ERROR() << "obj B0 missing, objNum: " << m_mapRadarDatas[m_idB0].ptrObjData->data_size
	// 						<< ", cur objNum: " << m_mapRadarDatas[m_idB0].ptrObjData->msg_datas.size();
	// 			LOG_FILE_PACKER_ERROR() << "obj B0 missing, objNum: " << m_mapRadarDatas[m_idB0].ptrObjData->data_size
	// 									<< ", cur objNum: " << m_mapRadarDatas[m_idB0].ptrObjData->msg_datas.size();
	// 			// << ", laneNum: " << m_mapRadarDatas[tId].ptrLaneData->data_size
	// 			// << ", cur laneNum: " << m_mapRadarDatas[tId].ptrLaneData->msg_datas.size();
	// 			LogStrings(*m_mapRadarDatas[m_idB0].ptrObjData, false);
	// 			if (bhasIdC0)
	// 			{
	// 				LogStrings(*m_mapRadarDatas[m_idC0].ptrObjData);
	// 			}
	// 		}
	// 		InsertData(m_idB0, 20);
	// 		m_mapRadarDatas[m_idB0].ptrObjData->header = msg->header;
	// 		m_mapRadarDatas[m_idB0].ptrObjData->msg_datas.push_back(*msg);
	// 		return -1;
	// 	}
	// 	else
	// 	{
	// 		if (!bhasIdB0 && !bhasIdC0)
	// 		{
	// 			// LOG_INFO() << "Waiting for Obj Status...";
	// 			return -1;
	// 		}
	// 	}
	// 	if (NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TLeftB0, msg->msg_name))
	// 	{
	// 		if (!bhasIdB0)
	// 		{
	// 			LOG_WARN() << "No B0_Obj_0A: " << LogString(*msg);
	// 			return -1;
	// 		}
	// 		m_mapRadarDatas[m_idB0].ptrObjData->msg_datas.push_back(*msg);
	// 		if (m_mapRadarDatas[m_idB0].ptrObjData->msg_datas.size() == m_mapRadarDatas[m_idB0].ptrObjData->data_size)
	// 		{
	// 			if (!bhasId && !bhasIdC0)
	// 			{
	// 				LOG_WARN() << "No C0 Data, only have B0 Data: " << LogString(*msg);
	// 				RemoveRadarData(m_idB0);
	// 				return -1;
	// 			}
	// 			auto &vec1 = m_mapRadarDatas[m_idB0].ptrObjData->msg_datas;
	// 			m_mapRadarDatas[tId].ptrObjData->msg_datas.insert(m_mapRadarDatas[tId].ptrObjData->msg_datas.end(), vec1.begin(), vec1.end());
	// 			RemoveRadarData(m_idB0);
	// 		}
	// 	}
	// 	if (
	// 		// NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TLeftB1, msg->msg_name) ||
	// 		NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TRightC0, msg->msg_name)
	// 		// NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TRightC1, msg->msg_name)
	// 	)
	// 	{
	// 		if (!bhasIdC0)
	// 		{
	// 			LOG_WARN() << "No C0_Obj_0A: " << LogString(*msg);
	// 			return -1;
	// 		}
	// 		m_mapRadarDatas[m_idC0].ptrObjData->msg_datas.push_back(*msg);
	// 		if (m_mapRadarDatas[m_idC0].ptrObjData->msg_datas.size() == m_mapRadarDatas[m_idC0].ptrObjData->data_size)
	// 		{
	// 			if (!bhasId)
	// 			{
	// 				InsertData(tId, 40);
	// 				m_mapRadarDatas[tId].ptrObjData->header = m_mapRadarDatas[m_idC0].ptrObjData->header;
	// 			}
	// 			auto &vec1 = m_mapRadarDatas[m_idC0].ptrObjData->msg_datas;
	// 			m_mapRadarDatas[tId].ptrObjData->msg_datas.insert(m_mapRadarDatas[tId].ptrObjData->msg_datas.end(), vec1.begin(), vec1.end());
	// 			RemoveRadarData(m_idC0);
	// 		}
	// 	}
	// 	if (bhasId)
	// 	{
	// 		if (m_mapRadarDatas[tId].ptrObjData->msg_datas.size() == m_mapRadarDatas[tId].ptrObjData->data_size)
	// 		{
	// 			return tId;
	// 		}
	// 	}
	// 	return -1;
	// }
	// virtual int PackedRadarDataCallback(const can_msgs::msg::CanMsgData::ConstSharedPtr msg) override
	// {
	// 	// status
	// 	uint32_t tId = 0x18FFBEB0; //
	// 	bool bhasId = CheckRadarDataValid(tId);
	// 	if (!msg->msg_name.compare(gk_radar5G4TRightC00A)) //
	// 	{
	// 		if (bhasId)
	// 		{
	// 			LOG_ERROR() << "5G4T obj msg missing, objNum: " << m_mapRadarDatas[tId].ptrObjData->data_size
	// 						<< ", cur objNum: " << m_mapRadarDatas[tId].ptrObjData->msg_datas.size();
	// 			LOG_FILE_PACKER_ERROR() << "obj missing, objNum: " << m_mapRadarDatas[tId].ptrObjData->data_size
	// 									<< ", cur objNum: " << m_mapRadarDatas[tId].ptrObjData->msg_datas.size();
	// 			// << ", laneNum: " << m_mapRadarDatas[tId].ptrLaneData->data_size
	// 			// << ", cur laneNum: " << m_mapRadarDatas[tId].ptrLaneData->msg_datas.size();
	// 			LogStrings(*m_mapRadarDatas[tId].ptrObjData);
	// 		}
	// 		m_mapMsgNames.clear();
	// 		ObjDataPtr objData = std::make_shared<RadarData>();
	// 		objData->header = msg->header;
	// 		objData->data_size = 21;
	// 		objData->msg_datas.push_back(*msg);
	// 		m_mapMsgNames.insert(msg->msg_name);
	// 		PackedRadarData tData;
	// 		tData.radarType = m_iRadarType;
	// 		tData.ptrObjData = objData;
	// 		m_mapRadarDatas[tId] = tData;
	// 		return -1;
	// 	}
	// 	else
	// 	{
	// 		if (!bhasId)
	// 		{
	// 			// LOG_INFO() << "Waiting for Obj Status...";
	// 			return -1;
	// 		}
	// 	}
	// 	if (
	// 		// NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TLeftB0, msg->msg_name) ||
	// 		// NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TLeftB1, msg->msg_name) ||
	// 		NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TRightC0, msg->msg_name) ||
	// 		NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TS5C0, msg->msg_name)
	// 		// NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TRightC1, msg->msg_name)
	// 	)
	// 	{
	// 		m_mapMsgNames.insert(msg->msg_name);
	// 		m_mapRadarDatas[tId].ptrObjData->msg_datas.push_back(*msg);
	// 	}
	// 	if (m_mapRadarDatas[tId].ptrObjData->msg_datas.size() == m_mapRadarDatas[tId].ptrObjData->data_size)
	// 	{
	// 		if (m_mapMsgNames.size() != m_mapRadarDatas[tId].ptrObjData->data_size)
	// 		{
	// 			LOG_ERROR() << "5G4T repeated obj msg, objNum: " << m_mapRadarDatas[tId].ptrObjData->data_size
	// 						<< ", cur objNum: " << m_mapMsgNames.size() ;
	// 			LOG_FILE_PACKER_ERROR() << "5G4T repeated obj msg, objNum: " << m_mapRadarDatas[tId].ptrObjData->data_size
	// 									<< ", cur objNum: " << m_mapMsgNames.size() ;
	// 			// << ", laneNum: " << m_mapRadarDatas[tId].ptrLaneData->data_size
	// 			// << ", cur laneNum: " << m_mapRadarDatas[tId].ptrLaneData->msg_datas.size();
	// 			LogStrings(*m_mapRadarDatas[tId].ptrObjData);
	// 			RemoveRadarData(tId);
	// 			return -1;
	// 		}
	// 		return tId;
	// 	}
	// 	return -1;
	// }
};

class RadarAC1000TPack : public RadarPackPolicy
{
public:
	RadarAC1000TPack(std::string ns, uint32_t radarType = ERadarType::RADAR_AC1000T) : RadarPackPolicy(ns, radarType) {};
	virtual ~RadarAC1000TPack() {};
	virtual int PackedRadarDataCallback(const can_msgs::msg::CanMsgData::ConstSharedPtr msg) override
	{

		// status
		uint32_t tId = 0x18FFBEB0; // begin with 67F: FC_Video_Data_General_A
		bool bhasId = CheckRadarDataValid(tId);
		if (!msg->msg_name.compare(gk_radar5G4TLeftB00A)) // FC_Video_Data_General_A
		{
			if (bhasId)
			{
				LOG_ERROR() << "obj missing, objNum: " << m_mapRadarDatas[tId].ptrObjData->data_size
							<< ", cur objNum: " << m_mapRadarDatas[tId].ptrObjData->msg_datas.size();
				// << ", laneNum: " << m_mapRadarDatas[tId].ptrLaneData->data_size
				// << ", cur laneNum: " << m_mapRadarDatas[tId].ptrLaneData->msg_datas.size();
			}
			ObjDataPtr objData = std::make_shared<RadarData>();
			objData->header = msg->header;
			objData->data_size = 10;
			objData->msg_datas.push_back(*msg);
			PackedRadarData tData;
			tData.radarType = m_iRadarType;
			tData.ptrObjData = objData;
			m_mapRadarDatas[tId] = tData;
			return -1;
		}
		else
		{
			if (!bhasId)
			{
				LOG_INFO() << "Waiting for AC1000T Obj Status...";
				return -1;
			}
		}
		if (NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TLeftB0, msg->msg_name) ||
			// NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TLeftB1, msg->msg_name) ||
			NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TRightC0, msg->msg_name)
			// NS_ZF_STRING_UTIL::IsSubStr(gk_radar5G4TRightC1, msg->msg_name)
		)
		{
			m_mapRadarDatas[tId].ptrObjData->msg_datas.push_back(*msg);
		}
		if (m_mapRadarDatas[tId].ptrObjData->msg_datas.size() == m_mapRadarDatas[tId].ptrObjData->data_size)
		{
			return tId;
		}
		return -1;
	}
};

class RadarNovaPack : public RadarPackPolicy
{
public:
	// m_mapRadarDatas status can id 60a, 61a
	RadarNovaPack(std::string ns, uint32_t radarType) : RadarPackPolicy(ns, radarType) {};
	virtual ~RadarNovaPack() {};
	virtual int PackedRadarDataCallback(const can_msgs::msg::CanMsgData::ConstSharedPtr msg) override
	{
		// status
		auto tId = msg->msg_id & 0xFF0;
		bool bhasId = CheckRadarDataValid(tId);
		if ((msg->msg_id & 0xF) == 0xA)
		{
			if (bhasId)
			{
				LOG_ERROR() << "obj missing, objNumTotal: " << m_mapRadarDatas[tId].ptrObjData->data_size
							<< ", objNumCurB: " << m_iObjNumCurB
							<< ", objNumCurC: " << m_iObjNumCurC
							<< ", objNumCurD: " << m_iObjNumCurD;
			}
			ObjDataPtr objData = std::make_shared<RadarData>();
			objData->header = msg->header;
			objData->data_size = this->GetValueBySigName(msg, gk_radarNovaObjNum);
			PackedRadarData tData;
			// tData.objNumTotal = objData->data_size;
			tData.ptrObjData = objData;
			m_mapRadarDatas[tId] = tData;
			if (!objData->data_size)
			{
				return tId;
			}
			return -1;
		}
		else
		{
			if (!bhasId)
			{
				LOG_INFO() << "Waiting for Nova Obj Status...";
				return -1;
			}
		}
		if ((msg->msg_id & 0xF) == 0xB)
		{
			m_mapRadarDatas[tId].ptrObjData->msg_datas.push_back(*msg);
			m_iObjNumCurB += 1;
		}
		else if ((msg->msg_id & 0xF) == 0xC)
		{
			m_mapRadarDatas[tId].ptrObjData->msg_datas.push_back(*msg);
			m_iObjNumCurC += 1;
		}
		else if ((msg->msg_id & 0xF) == 0xD)
		{
			m_mapRadarDatas[tId].ptrObjData->msg_datas.push_back(*msg);
			m_iObjNumCurD += 1;
		}
		if (m_iObjNumCurB == m_mapRadarDatas[tId].ptrObjData->data_size &&
			m_iObjNumCurC == m_mapRadarDatas[tId].ptrObjData->data_size &&
			m_iObjNumCurD == m_mapRadarDatas[tId].ptrObjData->data_size)
		{
			return tId;
		}
		return -1;
	}

	bool RemoveRadarData(const uint32_t &id) override
	{
		m_iObjNumCurB = 0;
		m_iObjNumCurC = 0;
		m_iObjNumCurD = 0;
		return RadarPackPolicy::RemoveRadarData(id);
	}

private:
	uint32_t m_iObjNumCurB = {};
	uint32_t m_iObjNumCurC = {};
	uint32_t m_iObjNumCurD = {};
};

END_NS_ZF_HAL_SYNC

#endif /* ZF_HAL_RADAR_PACK_POLICY_H_ */
