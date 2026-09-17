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
 * @brief Defines the DBC file Iterator, including multi messages.
 * @run
 */

#ifndef ZF_HAL_DBCTREE_H_
#define ZF_HAL_DBCTREE_H_

#include <vector>
#include <iosfwd>

#include "zf_hal_canmsg_parser/dbc_message.h"

/**
 * This is the Top class of the dbclib and the interface to the user.
 * It enables its user to iterate over the Messages of a DBC-File
 */

// /**
//  * @namespace NS_ZF::driver::canbus
//  */
BEGIN_NS_ZF_DRIVER_CANBUS

class DBCIterator
{

public:
	// typedef std::vector<Message> messages_t;
	using messages_map = std::unordered_map<std::string, Message>;// Idstr, msg
	using uint_str_map = std::unordered_map<uint32_t, std::string>;// uint Id, Idstr
	using name_map = std::unordered_map<std::string, std::string>;
	using pgn_id_map = std::unordered_map<std::string, std::vector<uint32_t>>; // one pgn with multi canID
	using const_iterator = messages_map::const_iterator;

	// Constructors taking either a File or a Stream of a DBC-File
	explicit DBCIterator(const std::string &filePath);
	explicit DBCIterator(std::istream &stream);

	/**
	 * @brief Registers the given message with the frame.
	 */
	void AddMessage(const Message &msg);

	/**
	 * @brief Removes the message associated with the given Hex Id
	 */
	void RemoveMessage(const std::string &canid);
	bool CheckPGN(const std::string &pgn);
	
	/**
	 * @brief Modify Id only when: pgn found but the srcId(not empty), and id can not be found in dbc.
	 * @param id The output can id
	 * @param pgn the pgn to find
	 * @param srcId the src id to find, usually get by Func Byte::EcuFromCanId
	 * @return if id was rewrite or not.
	 */
	bool ModifyId(uint32_t &id, const std::string &pgn, const std::string &srcId);
	bool CheckName(const std::string &name);

	/*
	 * Functionality to access the Messages parsed from the File
	 * either via the iterators provided by begin() and end() or by
	 * random access operator[]
	 */
	const_iterator begin() const { return m_mapMessage.begin(); }
	const_iterator end() const { return m_mapMessage.end(); }
	// MessagesMap::const_reference operator[](std::size_t elem) const {
	/**
	 * @brief Fetches the signal with the given name.
	 * @see at
	 */
	const Message at(const std::string &pgn, const std::string &srcId = "");
	const Message getMsgByName(const std::string &name);
	const Message &operator[](const std::string &pgn) const;

private:
	void init(std::istream &stream);
	// This list contains all the messages which got parsed from the DBC-File
	messages_map m_mapMessage; // canID message
	uint_str_map m_mapId; // canID message
	name_map m_mapMessageName; // canID message
	pgn_id_map m_mapPgnCanID;  // PGN canID
};

END_NS_ZF_DRIVER_CANBUS

#endif /* ZF_HAL_DBCTREE_H_ */
