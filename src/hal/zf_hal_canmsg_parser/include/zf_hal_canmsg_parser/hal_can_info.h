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
 * @brief Defines the can info interface for ros node and can parser pack.
 * @run
 */

#ifndef ZF_HAL_CANMSG_INFO_H_
#define ZF_HAL_CANMSG_INFO_H_

#include <string>
#include <memory> //unique_ptr

#include "zf_hal_canmsg_parser/dbc_iterator.h"
#include "zf_global/common/zf_global_macros.h"

// /**
//  * @namespace NS_ZF::driver::canbus
//  */
BEGIN_NS_ZF_DRIVER_CANBUS

class CanDataInfo
{
public:
	enum class PGNType
	{
		CUSTOM,	   // CanId < 65536, such as radar
		BROADCAST, // 1939 PGN Begin with 0xF: > 240
		P2P,	   // 1939 PGN Begin with not 0xF: < 240 point to point;
		TPCM,	   // 1939 PGN: 0xEC00
		TPDT	   // 1939 PGN: 0xEB00
	};

	CanDataInfo(uint32_t canId, std::vector<uint8_t> data, std::string dbcName);
	~CanDataInfo();

	uint32_t GetCanID() const;
	PGNType GetPGNType() const;
	std::string GetPGN() const;
	std::string GetDBCName() const;
	std::string GetEcuAddr() const;
	std::vector<uint8_t> GetData() const;
	void SetData(const std::vector<uint8_t> &data);
	std::vector<std::string> GetSigNames() const;
	void SetSigNames(const std::vector<std::string> &names);
	void SetTimeStamp(const uint32_t &sec, const uint32_t &nanosec);
	timeval GetTimeStamp() const {return m_stamp;};
	bool IsIgnoreCounter() const { return m_bIgnoreCounter; };
	void SetIgnoreCounter(const bool &v) { m_bIgnoreCounter = v; };
	bool IsIgnoreChecksum() const { return m_bIgnoreChecksum; };
	void SetIgnoreChecksum(const bool &v) { m_bIgnoreChecksum = v; };
	static bool CalChecksum(uint32_t val, uint32_t address, const std::vector<uint8_t> &d); // compare if checksum == val
	static uint32_t CalChecksum(uint32_t address, const std::vector<uint8_t> &d);				   // d include counter
	// static uint32_t CalChecksum(uint32_t address, const std::vector<uint8_t> &d, uint8_t counter); // d exclude counter and checksum
	uint32_t CalChecksum(uint8_t counter = 0x0); 
	void PrintInfo() const;

private:
	void Init();
	uint32_t m_id;
	std::string m_sEcuAddress; // From which ECU came this message
	std::string m_sPGN;
	std::string m_sDBCName;
	std::vector<std::string> m_sSigNames;
	PGNType m_pgnType;
	std::vector<uint8_t> m_data;
	bool m_bIgnoreCounter = {};
	bool m_bIgnoreChecksum = {};
	timeval m_stamp;// sec and nanosec here; for notice
	// uint32_t m_sec;
	// uint32_t m_nanosec;
};

END_NS_ZF_DRIVER_CANBUS

#endif /* ZF_HAL_CANMSG_INFO_H_ */