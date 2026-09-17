/*
 * dbctree.cpp
 *
 *  Created on: 04.10.2013
 *      Author: downtimes
 */

#include "zf_hal_canmsg_parser/dbc_iterator.h"

#include <limits>
#include <fstream>
#include <stdexcept>

#include "zf_hal_can_driver/byte.h"

BEGIN_NS_ZF_DRIVER_CANBUS

DBCIterator::DBCIterator(const std::string &filePath)
{
	std::ifstream file(filePath);
	if (file)
	{
		init(file);
	}
	else
	{
		throw std::invalid_argument("The File could not be opened");
	}
	file.close();
}

DBCIterator::DBCIterator(std::istream &stream)
{
	init(stream);
}

void DBCIterator::AddMessage(const Message &msg)
{
	m_mapMessage.insert(std::make_pair(msg.getIdStr(), msg));
	m_mapId.insert(std::make_pair(msg.getId(), msg.getIdStr()));
	m_mapMessageName.insert(std::make_pair(msg.getName(), msg.getIdStr()));
	if (CheckPGN(msg.getPGN()))
	{
		auto msgId = msg.getId();
		auto &vec = m_mapPgnCanID[msg.getPGN()];
		if ((msgId & 0xFFFF) != 0xFEFE)
		{
			vec.push_back(msgId);
		}
		else{
			vec.insert(vec.begin(), msgId);
		}
	}
	else
	{
		m_mapPgnCanID[msg.getPGN()] = {msg.getId()};
	}
}

void DBCIterator::RemoveMessage(const std::string &canid)
{
	auto ite = m_mapMessage.find(canid);
	if (ite == m_mapMessage.end())
	{
		std::string excepText = "Cannot remove message with can_id \"" + canid;
		throw std::out_of_range(excepText);
	}
	m_mapMessage.erase(ite);
}

bool DBCIterator::CheckPGN(const std::string &pgn)
{
	auto ite = m_mapPgnCanID.find(pgn);
	if (ite == m_mapPgnCanID.end())
	{
		return false;
	}
	return true;
}

bool DBCIterator::ModifyId(uint32_t &id, const std::string &pgn, const std::string &srcId)
{
	auto ite = m_mapPgnCanID.find(pgn);
	if (ite == m_mapPgnCanID.end())
	{
		return false;
	}
	auto idVec = m_mapPgnCanID.at(pgn);
	if (idVec.size())
	{
		if (!srcId.empty())
		{
			for (auto &eachId : idVec)
			{
				if (id == eachId)
				{
					return false;
				}
				// auto ecuDbc = Byte::EcuFromCanId(eachId);
				// if (srcId.length() < ecuDbc.length())
				// {
				// 	ecuDbc = ecuDbc.substr(ecuDbc.length() - srcId.length());
				// }
				// if (!ecuDbc.compare(srcId))
				// {
				// 	return false;
				// }
			}
		}
		id = idVec[0];
		return true;
	}
	return false;
}

bool DBCIterator::CheckName(const std::string &name)
{
	auto ite = m_mapMessageName.find(name);
	if (ite == m_mapMessageName.end())
	{
		return false;
	}
	return true;
}

const Message DBCIterator::at(const std::string &pgn, const std::string &ecuId) // source id
{
	auto idVec = m_mapPgnCanID.at(pgn);
	if (idVec.size())
	{
		if (!ecuId.empty())
		{
			for (auto &id : idVec)
			{
				auto ecuDbc = Byte::EcuFromCanId(id);
				if (ecuId.length() < ecuDbc.length())
				{
					ecuDbc = ecuDbc.substr(ecuDbc.length() - ecuId.length());
				}
				if (!ecuDbc.compare(ecuId))
				{
					return m_mapMessage.at(m_mapId[id]);
				}
			}
		}
		auto idStr = m_mapId.at(idVec[0]);
		return m_mapMessage.at(idStr);
	}
	return Message(); // Normally, this can not reached
}

const Message DBCIterator::getMsgByName(const std::string &name)
{
	return m_mapMessage.at(m_mapMessageName.at(name));
}

const Message &DBCIterator::operator[](const std::string &pgn) const
{
	auto idVec = m_mapPgnCanID.at(pgn);
	if (idVec.size())
	{
		auto idStr = m_mapId.at(idVec[0]);
		return m_mapMessage.at(idStr);
	}
	return Message(); // Normally, this can not reached
}

void DBCIterator::init(std::istream &stream)
{
	m_mapMessage.clear();
	// std::vector<Message> messages;
	do
	{
		Message msg;
		stream >> msg;
		if (stream.fail())
		{
			stream.clear();
			stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		}
		else
		{
			// messages.push_back(msg);
			AddMessage(msg);
		}
	} while (!stream.eof());
	// messageList.insert(messageList.begin(), messages.begin(), messages.end());
}

END_NS_ZF_DRIVER_CANBUS
