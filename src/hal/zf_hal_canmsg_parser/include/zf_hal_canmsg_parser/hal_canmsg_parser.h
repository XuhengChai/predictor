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
 * @brief Defines the can parser pack functions.
 * @run
 */

#ifndef ZF_HAL_CANMSG_PARSER_H_
#define ZF_HAL_CANMSG_PARSER_H_

#include <string>
#include <memory> //unique_ptr

#include "zf_hal_canmsg_parser/dbc_iterator.h"
#include "zf_hal_canmsg_parser/hal_can_info.h"
#include "zf_hal_canmsg_parser/hal_can_pgn_define.h"

#include "zf_global/common/zf_global_macros.h"
#include "zf_hal_can_driver/byte.h"

// /**
//  * @namespace NS_ZF::driver::canbus
//  */
BEGIN_NS_ZF_DRIVER_CANBUS

class Byte;

struct DBCSignalOptions
{
	std::string dbcName;
	uint32_t address;
	std::string sigName;
	double value;
};

struct SignalPackValue
{
	std::string name;
	double value;
};

/* PGN: 0x00EB00 - Storing the Transport Protocol Data Transfer from the reading process */
struct TP_DT
{
	uint8_t sequence_number;	  /* When this sequence number is the same as number_of_packages from TP_CM, then we have our complete message */
	std::vector<uint8_t> data;	  /* This is the collected data we are going to send. Also we are using this as a filler */
	std::string from_ecu_address; /* From which ECU came this message */
	bool operator<(const TP_DT &other) const
	{
		return sequence_number < other.sequence_number;
	}
};
/* PGN: 0x00EC00 - Storing the Transport Protocol Connection Management from the reading process */
struct TP_CM
{
	uint8_t control_byte;				  /* What type of message are we going to send */
	uint16_t total_message_size;		  /* Total bytes our complete message includes - 9 to 1785 */
	uint8_t number_of_packages;			  /* How many times we are going to send packages via TP_DT - 2 to 224 because 1785/8 is 224 rounded up */
	uint32_t PGN_of_the_packeted_message; /* Our message is going to activate a PGN */
	std::string from_ecu_address;		  /* From which ECU came this message */
	bool isDone;
	std::vector<TP_DT> data;
};

class CanMsgParser
{
public:
	using DBC_map = std::unordered_map<std::string, std::shared_ptr<DBCIterator>>;
	using TPCM_map = std::unordered_map<std::string, TP_CM>; //  From which ECU came this message, TP_CM
	using SignalPPMap = std::unordered_map<std::string, double>;

	using const_iterator = DBC_map::const_iterator;
	// using CanDataType = Byte;
	using CanDataType = uint8_t;

	// Constructors taking either a File or a Stream of a DBC-File
	// CanMsgParser();
	~CanMsgParser();

	std::string ExtractFileName(const std::string &name, bool withExtension = false);

	bool CheckValid(const std::string &dbcName, const std::string &pgn = "", const std::string &sigName = "");

	/**
	 * @brief Registers the DBC Parser with the given name and .dbc file path.
	 */
	std::string AddDBCParser(const std::string &filePath);
	void AddDBCParser(const std::string &name, const std::string &filePath);
	std::shared_ptr<DBCIterator> GetDBCParser(const std::string &name);
	void PrintDBCParser(const std::string &name);

	/**
	 * @brief Removes the DBC Parser associated with the given name
	 */
	void RemoveDBCParser(const std::string &name);

	NS_ZF::ErrorCode Parse(SignalPPMap &sigVals, const CanDataInfo &info);
	/**
	 * @brief Multi Sigs Parser, sigName = "". can also specify the sigName
	 */
	NS_ZF::ErrorCode Parse(SignalPPMap &sigVals, const std::vector<CanDataType> &dat, DBCSignalOptions &dbcOpt);
	/**
	 * @brief Single Sig Parser, must specify the sigName;
	 */
	NS_ZF::ErrorCode Parse(const std::vector<CanDataType> &dat, DBCSignalOptions &dbcOpt);

	NS_ZF::ErrorCode Parse(SignalPPMap &sigVals, const CanDataInfo &info, const std::string &dbcName);
	/**
	 * @brief Multi Sigs Parser, no sigName
	 */
	NS_ZF::ErrorCode Parse(SignalPPMap &sigVals, const std::vector<CanDataType> &dat, const std::string &dbcName, const uint32_t &address);
	NS_ZF::ErrorCode Parse(SignalPPMap &sigVals, const std::vector<CanDataType> &dat, const std::string &dbcName, const std::string &pgn, const std::string &ecuId);

	/**
	 * @brief Single Sigs Parser
	 */
	double Parse(const std::vector<CanDataType> &dat, const std::string &dbcName, const uint32_t &address, const std::string &sigName);

	NS_ZF::ErrorCode ParseImpl(SignalPPMap &sigVals, const CanDataInfo &info);

	NS_ZF::ErrorCode Pack(std::vector<CanMsgParser::CanDataType> &data, const SignalPPMap &signals, const std::string &dbcName, uint32_t address);

	template <typename T>
	NS_ZF::ErrorCode GetValue(double &val, const std::vector<T> &data, const Signal &sig);

	template <typename T>
	NS_ZF::ErrorCode GetValue(double &val, const std::vector<T> &data, const Signal &sig, uint32_t addr,
							  bool ignoreCounter = false, bool ignoreChecksum = false);

	/*
	 * @brief check if the value is in [lower, upper], if not , round it to bound
	 */
	template <typename T>
	static bool BoundedValue(T &val, T lower, T upper);

	int64_t GetRawValue(const std::vector<uint8_t> &data, const Signal &sig);
	void SetValue(std::vector<uint8_t> &data, const Signal &sig, int64_t ival);

	int64_t GetRawValue(const std::vector<Byte> &data, const Signal &sig);
	void SetValue(std::vector<Byte> &data, const Signal &sig, int64_t ival);

private:
	void MergeTPCMData(std::vector<uint8_t> &mergedData, const TP_CM &data);
	bool UpdateCounterGeneric(uint32_t addr, int64_t v, int cnt_size);
	// Variable Declaration
	const std::string filePath;
	DBC_map m_mapDBCParser;
	TPCM_map m_mapTPCM;
	std::unordered_map<uint32_t, uint32_t> m_mapPackCounters;
	std::unordered_map<uint32_t, uint32_t> m_mapParserCounters;
	std::unordered_map<uint32_t, uint32_t> m_mapParserCountersFail;

private:
	DECLARE_SINGLETON(CanMsgParser)
};

template <typename T>
NS_ZF::ErrorCode CanMsgParser::GetValue(double &val, const std::vector<T> &data, const Signal &sig)
{
	return GetValue<T>(val, data, sig, 0, true, true);
}

template <typename T>
NS_ZF::ErrorCode CanMsgParser::GetValue(double &val, const std::vector<T> &data, const Signal &sig, uint32_t addr,
										bool ignoreCounter /* = false*/, bool ignoreChecksum /* = false*/)
{
	int64_t tmp = GetRawValue(data, sig);
	bool ignoreBound = true;
	if (tmp ^ ((1ULL << sig.getLength()) - 1))
	{
		ignoreBound = false;
	}
	if (sig.getSign() == Sign::SIGNED)
	{
		tmp -= ((tmp >> (sig.getLength() - 1)) & 0x1) ? (1ULL << sig.getLength()) : 0; //  If size = 16: -768 = 64768 - 65536(2^16);
	}

	bool checksum_failed = false;
	if (!ignoreChecksum)
	{
		if (sig.getSignalType() == Signal::SignalType::CHECKSUM)
		{
			if (!CanDataInfo::CalChecksum(tmp, addr, data))
			{
				// LOG_INFO() << "CalChecksum wrong" << "--" << tmp  << "--" << CanDataInfo::CalChecksum(addr, data);
				checksum_failed = true;
			}
		}
	}
	bool counter_failed = false;
	if (!ignoreCounter)
	{
		if (sig.getSignalType() == Signal::SignalType::COUNTER)
		{
			auto pgn = Byte::PGNFromCanId(addr);
			if (!pgn.compare(gk_pgnTSC1))
			{
				// LOG_DEBUG() << gk_pgnTSC1 << ", TMP IS " << tmp << ", and addr is " << Byte::Int2Hex(addr);
				counter_failed = !UpdateCounterGeneric(addr, tmp, 3); // 1 << 3 - 1 = 7
			}
			else if (!pgn.compare(gk_pgnXBR))
			{
				counter_failed = !UpdateCounterGeneric(addr, tmp, sig.getLength());
			}
		}
	}
	if (checksum_failed || counter_failed)
	{
		LOG_ERROR() << "message checks failed: 0x" << Byte::Int2Hex(addr) << "#" << checksum_failed << "--" << counter_failed
					<< "--" << tmp << "--" << CanDataInfo::CalChecksum(addr, data);
		return NS_ZF::ErrorCode::CAN_PARSE_CHECK_FAILED;
	}

	val = tmp * sig.getFactor() + sig.getOffset();
	if (ignoreBound)
	{
		return ErrorCode::OK;
	}
	// BrkAppPrssHghRngRrAxl2LeftWheel BrkAppPrssHghRngRrAxl2RghtWheel EstEngPrsticLossesPercentTorque
	// if (!sig.getName().compare("EstEngPrsticLossesPercentTorque") || (!sig.getName().compare("BrkAppPrssHghRngRrAxl2LeftWheel")))
	// {
	// 	return ErrorCode::OK;
	// }
	bool inBound = BoundedValue(val, sig.getMinimum(), sig.getMaximum());
	if (!inBound)
	{
		LOG_WARN() << "signal not in bound value: " << sig.getName().c_str() << ", val: " << tmp * sig.getFactor() + sig.getOffset()
				   << ", Minimum: " << sig.getMinimum() << ", Maximum: " << sig.getMaximum();
		return NS_ZF::ErrorCode::CAN_PARSE_OUT_OF_BOUND;
	}
	// printf("RAW_VAL:%s, %ld , val %f , bound val %f\n", sig.getName().c_str(), tmp, val, boundVal);
	return NS_ZF::ErrorCode::OK;
}

template <typename T>
bool CanMsgParser::BoundedValue(T &val, T lower, T upper)
{
	if (lower > upper)
	{
		return false;
	}
	if ((lower - val) > 1e-10)
	{
		// LOG_ERROR() << "BoundedValue val--" << val << "--" << lower << "--" << lower - val;
		val = lower;
		return false;
	}
	if ((val - upper) > 1e-10)
	{
		val = upper;
		return false;
	}
	return true;
}

END_NS_ZF_DRIVER_CANBUS

#endif /* ZF_HAL_CANMSG_PARSER_H_ */