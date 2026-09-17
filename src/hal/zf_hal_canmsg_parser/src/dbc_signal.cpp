/*
 * signal.cpp
 *
 *  Created on: 04.10.2013
 *      Author: downtimes
 */

#include "zf_hal_canmsg_parser/dbc_signal.h"

#include <istream>
#include <sstream>
#include <limits>
#include <iterator>
#include <algorithm>
#include <vector>

BEGIN_NS_ZF_DRIVER_CANBUS

static const int32_t gk_byteLen = 8; // according to ISO-11891-1 8 bits for each byte
static const uint16_t gk_lookupTable[8] = {7, 6, 5, 4, 3, 2, 1, 0};
static const char gk_counter[] = "Counter";
static const char gk_checksum[] = "CheckSum";
static const char gk_message[] = "message";

std::string &trim(std::string &str, const std::string &toTrim = " ")
{
	std::string::size_type pos = str.find_last_not_of(toTrim);
	if (pos == std::string::npos)
	{
		str.clear();
	}
	else
	{
		str.erase(pos + 1);
		str.erase(0, str.find_first_not_of(toTrim));
	}
	return str;
}

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems)
{
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim))
	{
		elems.push_back(item);
	}
	return elems;
}

std::vector<std::string> split(const std::string &s, char delim)
{
	std::vector<std::string> elems;
	split(s, delim, elems);
	return elems;
}

std::istream &operator>>(std::istream &in, Signal &sig)
{
	int c = in.peek();
	if ('B' == c)
	{
		in.setstate(std::ios_base::failbit);
		return in;
	}
	std::string line;
	std::getline(in, line);
	if (!line.empty() && *line.rbegin() == '\r')
		line.erase(line.length() - 1, 1);
	if (line.empty())
	{
		in.setstate(std::ios_base::failbit);
		return in;
	}

	std::istringstream sstream(line);
	std::string preamble;
	sstream >> preamble;
	// Check if we are actually reading a Signal otherwise fail the stream
	if (preamble != "SG_")
	{
		sstream.setstate(std::ios_base::failbit);
		return in;
	}

	// Parse the Signal Name
	sstream >> sig.name;

	std::string multi;
	sstream >> multi;

	// This case happens if there is no Multiplexor present
	if (multi == ":")
	{
		sig.multiplexor = Multiplexor::NONE;
		// Case with multiplexor
	}
	else
	{
		if (multi == "M")
		{
			sig.multiplexor = Multiplexor::MULTIPLEXOR;
		}
		else
		{
			// The multiplexor looks like that 'm12' so we ignore the m and parse it as integer
			std::istringstream multstream(multi);
			multstream.ignore(1);
			unsigned short multiNum;
			multstream >> multiNum;
			sig.multiplexor = Multiplexor::MULTIPLEXED;
			sig.multiplexNum = multiNum;
		}
		// ignore the next thing which is a ':'
		sstream >> multi;
	}

	sstream >> sig.startBit;
	sstream.ignore(1);
	sstream >> sig.length;
	sstream.ignore(1);

	std::string t_name = sig.name;
	std::transform(t_name.begin(), t_name.end(), t_name.begin(), ::tolower);
	std::string t_counter = gk_counter;
	std::transform(t_counter.begin(), t_counter.end(), t_counter.begin(), ::tolower);
	std::string t_checksum = gk_checksum;
	std::transform(t_checksum.begin(), t_checksum.end(), t_checksum.begin(), ::tolower);
	sig.type = Signal::SignalType::DEFAULT;
	// TODO: for special counter whic sig.length is 2, change code if needed.
	if (
		((t_name.find(t_counter) != std::string::npos) && (sig.length == 4)) ||
		((t_name.find(t_counter) != std::string::npos) && (t_name.find(gk_message) != std::string::npos)))
	{
		sig.type = Signal::SignalType::COUNTER;
	}
	else if (t_name.find(t_checksum) != std::string::npos)
	{
		sig.type = Signal::SignalType::CHECKSUM;
	}

	int order;
	sstream >> order;
	/************
	 *  7	6	5	4	3	2	1	0
	 *
	0 	7, 	6, 	5, 	4, 	3, 	2, 	1, 	0,
	1	15, 14, 13, 12, 11, 10, 9, 	8,
	2	23, 22, 21, 20, 19, 18, 17, 16,
	3	31, 30, 29, 28, 27, 26, 25, 24,
	4	39, 38, 37, 36, 35, 34, 33, 32,
	5	47, 46, 45, 44, 43, 42, 41, 40,
	6	55, 54, 53, 52, 51, 50, 49, 48,
	 ***********/
	if (order == 0)
	{
		sig.order = ByteOrder::MOTOROLA;	   // If startBit = 32, len = 16;
		sig.msb.x = sig.startBit / gk_byteLen; // x = 4;
		sig.msb.y = sig.startBit % gk_byteLen; // y = 0;
		sig.msb.value = sig.startBit;
		int t_endBit = sig.msb.x * 8 + gk_lookupTable[sig.msb.y] + sig.length - 1; // t_endBit = 54
		sig.lsb.x = t_endBit / gk_byteLen;										   // x = 6
		sig.lsb.y = gk_lookupTable[t_endBit % gk_byteLen];						   // y = 1
		sig.lsb.value = sig.lsb.x * 8 + sig.lsb.y;
		// auto it = find(be_bits.begin(), be_bits.end(), sig.start_bit);
		// std::cout << "begi "<< (it - be_bits.begin()) << "and " <<(it - be_bits.begin()) + sig.size - 1 << std::endl;
		// sig.lsb = be_bits[(it - be_bits.begin()) + sig.size - 1];
		// sig.msb = sig.startBit;
	}
	else
	{
		// little_endian; cal begin and end row and col
		sig.order = ByteOrder::INTEL;		   // If startBit = 32, len = 16;
		sig.lsb.x = sig.startBit / gk_byteLen; // x = 4;
		sig.lsb.y = sig.startBit % gk_byteLen; // y = 0;
		sig.lsb.value = sig.startBit;
		int t_endBit = sig.startBit + sig.length - 1; // t_endBit = 47
		sig.msb.x = t_endBit / gk_byteLen;			  // x = 5
		sig.msb.y = t_endBit % gk_byteLen;			  // y = 7
		sig.msb.value = t_endBit;

		// sig.lsb = sig.startBit;
		// sig.msb = sig.startBit + sig.length - 1;
	}

	char sign;
	sstream >> sign;
	if (sign == '+')
	{
		sig.sign = Sign::UNSIGNED;
	}
	else
	{
		sig.sign = Sign::SIGNED;
	}

	sstream.ignore(std::numeric_limits<std::streamsize>::max(), '(');
	sstream >> sig.factor;
	sstream.ignore(1);
	sstream >> sig.offset;
	sstream.ignore(1);

	sstream.ignore(std::numeric_limits<std::streamsize>::max(), '[');
	sstream >> sig.minimum;
	sstream.ignore(1);
	sstream >> sig.maximum;
	sstream.ignore(1);

	std::string unit;
	sstream >> unit;
	sig.unit = trim(unit, "\"");

	std::string to;
	sstream >> to;
	// LOG_INFO() << "sstream.str() is "<<sstream.str().c_str();
	std::vector<std::string> toStrings = split(to, ',');
	std::move(toStrings.begin(), toStrings.end(), std::inserter(sig.to, sig.to.begin()));

	return in;
}

END_NS_ZF_DRIVER_CANBUS