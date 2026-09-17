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
 * @brief Defines the DBC file message, including multi signals.
 * @run
 */

#ifndef ZF_HAL_DBC_MESSAGE_H_
#define ZF_HAL_DBC_MESSAGE_H_

#include <string>
#include <unordered_map>
#include <iosfwd>
#include <cstdint>
#include <set>

#include "zf_hal_canmsg_parser/dbc_signal.h"

// /**
//  * @namespace NS_ZF::driver::canbus
//  */
BEGIN_NS_ZF_DRIVER_CANBUS

/**
 * Class representing a Message in the DBC-File. It allows its user to query
 * Data and to iterate over the Signals contained in the Message
 */
class Message
{
public:
	using signals_map = std::unordered_map<std::string, Signal>;
	using const_iterator = signals_map::const_iterator;
	// typedef signals_map::const_iterator const_iterator;
	// Overload of operator>> to enable parsing of Messages from streams of DBC-Files
	friend std::istream &operator>>(std::istream &in, Message &msg);

	// Getter functions for all the possible Data one can request from a Message
	std::string getName() const { return name; }
	std::string getIdStr() const { return idStr; }
	std::string getPGN() const;
	std::uint32_t getId() const { return id; }
	std::uint32_t getSize() const { return m_mapSignals.size(); }
	std::uint32_t getSizeValid() const;
	std::size_t getDlc() const { return dlc; }
	std::string getFrom() const { return from; }
	bool HasCounter() const { return m_bHasCounter; }
	bool HasChecksum() const { return m_bHasChecksum; }
	const Signal getCounterSig() const;
	const Signal getChecksumSig() const;
	std::set<std::string> getTo() const;
	void AddSignal(const Signal &signal);
	void RemoveSignal(const std::string &name);
	bool CheckSignal(const std::string &name) const;

	/*
	 * Functionality to access the Signals contained in this Message
	 * either via the iterators provided by begin() and end() or by
	 * random access operator[]
	 */
	const_iterator begin() const { return m_mapSignals.begin(); }
	const_iterator end() const { return m_mapSignals.end(); }
	const Signal at(const std::string &name) const
	{
		return m_mapSignals.at(name);
	}
	const Signal &operator[](const std::string &name) const
	{
		return m_mapSignals.at(name);
	}

private:
	// Name of the Message
	std::string name;
	std::string pgn;   // Parameter Group Number: 65278 (0x00FEFE) "FEFE"
	std::string idStr; // Parameter Group Number: 65278 (0x00FEFE) "FEFE"
	// The CAN-ID assigned to this specific Message
	std::uint32_t id;
	// The length of this message in Bytes. Allowed values are between 0 and 8
	std::size_t dlc;
	// String containing the name of the Sender of this Message if one exists in the DB
	std::string from;
	// List containing all Signals which are present in this Message
	signals_map m_mapSignals;
	bool m_bHasCounter = {};
	bool m_bHasChecksum = {};
	std::string m_sCounter;
	std::string m_sChecksum;
};

END_NS_ZF_DRIVER_CANBUS

#endif /* ZF_HAL_DBC_MESSAGE_H_ */
