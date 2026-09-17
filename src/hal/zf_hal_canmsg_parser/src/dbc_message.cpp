/*
 * message.cpp
 *
 *  Created on: 04.10.2013
 *      Author: downtimes
 */
#include "zf_hal_canmsg_parser/dbc_message.h"

#include <istream>
#include <sstream>
#include <limits>
#include <algorithm>
#include <iomanip>

#include "zf_hal_can_driver/byte.h"
#include "zf_hal_can_driver/hal_can_id.h"

BEGIN_NS_ZF_DRIVER_CANBUS

std::istream &operator>>(std::istream &in, Message &msg)
{
	std::string preamble;
	in >> preamble;
	// Check if we are actually reading a Message otherwise fail the stream
	if (preamble != "BO_")
	{
		in.setstate(std::ios_base::failbit);
		return in;
	}

	std::string line;
	std::string lineRight;
	std::getline(in, line);
	std::istringstream sstream(line);
	std::getline(sstream, line, ':');
	std::getline(sstream, lineRight, ':');

	sstream.clear();
	sstream.str(line);
	// Parse the message ID
	sstream >> msg.id;
	msg.id = CanId::Identifier(msg.id);

	size_t strWidth = msg.id > 65535 ? 8 : 4;
	msg.idStr = Byte::Int2Hex(msg.id, strWidth);
	msg.pgn = Byte::PGNFromCanId(msg.id);

	// Parse the name of the Message
	std::string name;
	sstream >> msg.name;

	sstream.clear();
	sstream.str(lineRight);
	// Parse the Messages length
	sstream >> msg.dlc;
	// Parse the sender;
	sstream >> msg.from;

	////Parse the message ID
	// in >> msg.id;

	////Parse the name of the Message
	// std::string name;
	// in >> name;
	// msg.name = name.substr(0, name.length() - 1);

	////Parse the Messages length
	// in >> msg.dlc;

	////Parse the sender;
	// in >> msg.from;

	// in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

	// As long as there is a Signal, parse the Signal
	while (in)
	{
		Signal sig;
		in >> sig;
		if (in)
		{
			// msg.signals.push_back(sig);
			msg.m_mapSignals.insert(std::make_pair(sig.getName(), sig));
			if (sig.getSignalType() == Signal::COUNTER)
			{
				msg.m_bHasCounter = true;
				msg.m_sCounter = sig.getName();
			}
			else if (sig.getSignalType() == Signal::CHECKSUM)
			{
				msg.m_bHasChecksum = true;
				msg.m_sChecksum = sig.getName();
			}
		}
	}

	in.clear();
	return in;
}

std::string Message::getPGN() const
{
	return pgn;
}

std::uint32_t Message::getSizeValid() const
{
	auto size = m_mapSignals.size();
	if (m_bHasCounter)
	{
		size--;
	}
	if (m_bHasChecksum)
	{
		size--;
	}
	return size;
}

const Signal Message::getCounterSig() const
{
	if (m_bHasCounter)
	{
		return m_mapSignals.at(m_sCounter);
	}
	return Signal();
}
const Signal Message::getChecksumSig() const
{
	if (m_bHasChecksum)
	{
		return m_mapSignals.at(m_sChecksum);
	}
	return Signal();
}
std::set<std::string> Message::getTo() const
{
	std::set<std::string> collection;
	for (auto sig : m_mapSignals)
	{
		auto toList = sig.second.getTo();
		collection.insert(toList.begin(), toList.end());
	}
	return collection;
}

void Message::AddSignal(const Signal &signal)
{
	m_mapSignals.insert(std::make_pair(signal.getName(), signal));
}

void Message::RemoveSignal(const std::string &name)
{
	auto ite = m_mapSignals.find(name);
	if (ite == m_mapSignals.end())
	{
		std::string excepText = "Cannot remove signal with name \"" + name +
								"\" from frame \"" + this->getName() + "\"";
		throw std::out_of_range(excepText);
	}
	m_mapSignals.erase(ite);
}

bool Message::CheckSignal(const std::string &name) const
{
	auto ite = m_mapSignals.find(name);
	if (ite == m_mapSignals.end())
	{
		return false;
	}
	return true;
}
END_NS_ZF_DRIVER_CANBUS