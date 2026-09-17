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
 * @brief Defines the DBC file signals, including multi signals.
 * @run
 */

#ifndef ZF_HAL_DBC_SIGNAL_H_
#define ZF_HAL_DBC_SIGNAL_H_

#include <string>
#include <iosfwd>
#include <set>

#include "zf_hal_can_driver/hal_can_driver_global.h"
#include "zf_global/common/zf_global_geometry.h"

using namespace NS_ZF_GEOMERY;

// /**
//  * @namespace NS_ZF::driver::canbus
//  */
BEGIN_NS_ZF_DRIVER_CANBUS

enum class ByteOrder
{
	MOTOROLA,
	INTEL
};

enum class Sign
{
	UNSIGNED,
	SIGNED
};

enum class Multiplexor
{
	NONE,
	MULTIPLEXED,
	MULTIPLEXOR
};

/**
 * This class represents a Signal contained in a Message of a DBC-File.
 * One can Query all the necessary information from this class to define
 * a Signal
 */
class Signal
{

public:
	enum SignalType
	{
		DEFAULT,
		COUNTER,
		CHECKSUM
	};

	// Overload of operator>> to allow parsing from DBC Streams
	friend std::istream &operator>>(std::istream &in, Signal &sig);
	typedef std::set<std::string> toList;

	// Getter for all the Values contained in a Signal
	std::string getName() const { return name; }
	ByteOrder getByteOrder() const { return order; }
	SignalType getSignalType() const { return type; }
	unsigned short getStartbit() const { return startBit; }
	unsigned short getLength() const { return length; }
	Vector2DValue getMsb() const { return msb; }
	Vector2DValue getLsb() const { return lsb; }
	Sign getSign() const { return sign; }
	double getMinimum() const { return minimum; }
	double getMaximum() const { return maximum; }
	double getFactor() const { return factor; }
	double getOffset() const { return offset; }
	std::string getUnit() const { return unit; }
	Multiplexor getMultiplexor() const { return multiplexor; }
	unsigned short getMultiplexedNumber() const { return multiplexNum; }
	toList getTo() const { return to; }

private:
	// The name of the Signal in the DBC-File
	std::string name;
	// The Byteorder of the Signal (@see: endianess)
	ByteOrder order;
	SignalType type;
	// The Startbit inside the Message of this Signal. Allowed values are 0-63
	unsigned short startBit;
	// The Length of the Signal. It can be anything between 1 and 64
	unsigned short length;
	Vector2DValue lsb; // Least Significant Byte: x is row and y is col, start from zero
	Vector2DValue msb; // Most Significant Byte: x is row and y is col, start from zero
	// If the Data contained in the Signal is signed or unsigned Data
	Sign sign;
	// Depending on the information given above one can calculate the minimum of this Signal
	double minimum;
	// Depending on the inforamtion given above one can calculate the maximum of this Signal
	double maximum;
	// The Factor for calculating the physical value: phys = digits * factor + offset
	double factor;
	// The offset for calculating the physical value: phys = digits * factor + offset
	double offset;
	// String containing an associated unit.
	std::string unit;
	// Contains weather the Signal is Multiplexed and if it is, multiplexNum contains multiplex number
	Multiplexor multiplexor;
	// Contains the multiplex Number if the Signal is multiplexed
	unsigned short multiplexNum;
	// Contains to which Control Units in the CAN-Network the Signal shall be sent
	toList to;
};

END_NS_ZF_DRIVER_CANBUS

#endif /* ZF_HAL_DBC_SIGNAL_H_ */
