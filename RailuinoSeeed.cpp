/*********************************************************************
 * RailuinoSeeed
 * Adopted from: Railuino - Hacking your Märklin
 *
 * Copyright (C) 2012 Joerg Pleumann
 * Copyright (C) 2022 Wolfgang Hammer
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * LICENSE file for more details.
 */

#include "RailuinoSeeed.h"
#include "mcp2515_can.h"

size_t printHex(Print &p, unsigned long hex, int digits)
{
	size_t size = 0;

	String s = String(hex, HEX);

	for (int i = s.length(); i < digits; i++)
	{
		size += p.print("0");
	}

	size += p.print(s);

	return size;
}

int parseHex(String &s, int start, int end, boolean *ok)
{
	int value = 0;

	for (int i = start; i < end; i++)
	{
		char c = s.charAt(i);

		if (c >= '0' && c <= '9')
		{
			value = 16 * value + c - '0';
		}
		else if (c >= 'a' && c <= 'f')
		{
			value = 16 * value + 10 + c - 'a';
		}
		else if (c >= 'A' && c <= 'F')
		{
			value = 16 * value + 10 + c - 'A';
		}
		else
		{
			*ok = false;
			return -1;
		}
	}

	return value;
}

#define SIZE 32

#define ulong unsigned long

// ===================================================================
// === TrackMessage ==================================================
// ===================================================================

void TrackMessage::clear()
{
	command = 0;
	hash = 0;
	response = false;
	length = 0;
	for (int i = 0; i < 8; i++)
	{
		data[i] = 0;
	}
}

size_t TrackMessage::printTo(Print &p) const
{
	size_t size = 0;

	size += printHex(p, hash, 4);
	size += p.print(response ? " R " : "   ");
	size += printHex(p, command, 2);
	size += p.print(" ");
	size += printHex(p, length, 1);

	for (int i = 0; i < length; i++)
	{
		size += p.print(" ");
		size += printHex(p, data[i], 2);
	}

	return size;
}

boolean TrackMessage::parseFrom(String &s)
{
	boolean result = true;

	clear();

	if (s.length() < 11)
	{
		return false;
	}

	hash = parseHex(s, 0, 4, &result);
	response = s.charAt(5) != ' ';
	command = parseHex(s, 7, 9, &result);
	length = parseHex(s, 10, 11, &result);

	if (length > 8)
	{
		return false;
	}

	if (s.length() < 11 + 3 * length)
	{
		return false;
	}

	for (int i = 0; i < length; i++)
	{
		data[i] = parseHex(s, 12 + 3 * i, 12 + 3 * i + 2, &result);
	}

	return result;
}

boolean TrackMessage::fromCanMsg(unsigned long aId, byte aExt, byte aRtr, byte aLen, byte *aBuf)
{
	clear();
	command = (aId >> 17) & 0xff;
	hash = aId & 0xffff;
	response = bitRead(aId, 16);
	length = aLen;

	for (int i = 0; i < length; i++)
	{
		data[i] = aBuf[i];
	}
	return true;
}

void TrackController::init(MCP_CAN &aCAN)
{
	mCAN = &aCAN;

	delay(500);

	TrackMessage message;

	message.clear();
	message.command = 0x1b;
	message.length = 0x05;
	message.data[4] = 0x11;

	sendMessage(message);
}

boolean TrackController::receiveMessage(TrackMessage &message)
{
	unsigned long t = millis();

	if (CAN_MSGAVAIL != mCAN->checkReceive())
	{
		return false;
	}

	uint32_t id;
	uint8_t ext;
	uint8_t rtr;
	uint8_t len;
	byte cdata[MAX_CHAR_IN_MESSAGE] = {0};

	// read data, len: data length, buf: data buf
	mCAN->readMsgBufID(mCAN->readRxTxStatus(), &id, &ext, &rtr, &len, cdata);

	message.fromCanMsg(id, ext, rtr, len, cdata);
	if (mDebug)
	{
		SERIAL_PORT_MONITOR.print("<== ");
		SERIAL_PORT_MONITOR.println(message);
	}
	return true;
}

boolean TrackController::sendMessage(TrackMessage &message)
{
	message.hash = mHash;

	const uint32_t id = ((uint32_t)message.command) << 17 | (uint32_t)message.hash;
	const uint8_t ext = 1;
	const uint8_t rtr = 0;

	if (mDebug)
	{
		SERIAL_PORT_MONITOR.print("==> ");
		SERIAL_PORT_MONITOR.println(message);
	}

	byte result = mCAN->sendMsgBuf(id, ext, rtr, message.length, message.data);
	if (mDebug)
	{
		SERIAL_PORT_MONITOR.print("  result ");
		SERIAL_PORT_MONITOR.println(result, HEX);
	}
	return result == CAN_OK;
}

boolean TrackController::exchangeMessage(TrackMessage &out, TrackMessage &in, word timeout)
{
	int command = out.command;

	if (!sendMessage(out))
	{
		if (true)
		{
			if (mDebug)
			{
				SERIAL_PORT_MONITOR.println(F("!!! Send error"));
				SERIAL_PORT_MONITOR.println(F("!!! Emergency stop"));
			}
			for (;;)
				;
		}
	}

	ulong time = millis();
	while (millis() < time + timeout)
	{
		in.clear();
		boolean result = receiveMessage(in);

		if (result && in.command == command && in.response)
		{
			return true;
		}
	}

	if (mDebug)
	{
		SERIAL_PORT_MONITOR.println(F("!!! Receive timeout"));
	}

	return false;
}

boolean TrackController::setPower(boolean power)
{
	TrackMessage message;

	if (power)
	{
		message.clear();
		message.command = 0x00;
		message.length = 0x07;
		message.data[4] = 9;
		message.data[6] = 0xD;

		exchangeMessage(message, message, 1000);

		message.clear();
		message.command = 0x00;
		message.length = 0x06;
		message.data[4] = 8;
		message.data[5] = 7;

		exchangeMessage(message, message, 1000);
	}

	message.clear();
	message.command = 0x00;
	message.length = 0x05;
	message.data[4] = power ? 0x01 : 0x00;

	return exchangeMessage(message, message, 1000);
}

boolean TrackController::setPower2(boolean power)
{
	TrackMessage message;

	message.clear();
	message.command = 0x00;
	message.length = 0x05;
	message.data[4] = power ? 0x01 : 0x00;

	return sendMessage(message);
}

boolean TrackController::getPower2(void)
{
	TrackMessage message;

	message.clear();
	message.command = 0x00;
	message.length = 0x04;

	return sendMessage(message);
}

boolean TrackController::setLocoDirection(word address, byte direction)
{
	TrackMessage message;

	message.clear();
	message.command = 0x00;
	message.length = 0x05;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);
	message.data[4] = 0x03;

	exchangeMessage(message, message, 1000);

	message.clear();
	message.command = 0x05;
	message.length = 0x05;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);
	message.data[4] = direction;

	return exchangeMessage(message, message, 1000);
}

boolean TrackController::toggleLocoDirection(word address)
{
	return setLocoDirection(address, DIR_CHANGE);
}

boolean TrackController::setLocoSpeed(word address, word speed)
{
	TrackMessage message;

	message.clear();
	message.command = 0x04;
	message.length = 0x06;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);
	message.data[4] = highByte(speed);
	message.data[5] = lowByte(speed);

	return exchangeMessage(message, message, 1000);
}

boolean TrackController::accelerateLoco(word address)
{
	word speed;

	if (getLocoSpeed(address, &speed))
	{
		speed += 77;
		if (speed > 1023)
		{
			speed = 1023;
		}

		return setLocoSpeed(address, speed);
	}

	return false;
}

boolean TrackController::decelerateLoco(word address)
{
	word speed;

	if (getLocoSpeed(address, &speed))
	{
		speed -= 77;
		if (speed > 32767)
		{
			speed = 0;
		}

		return setLocoSpeed(address, speed);
	}

	return false;
}

boolean TrackController::setLocoFunction(word address, byte function, byte power)
{
	TrackMessage message;

	message.clear();
	message.command = 0x06;
	message.length = 0x06;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);
	message.data[4] = function;
	message.data[5] = power;

	return exchangeMessage(message, message, 1000);
}

boolean TrackController::toggleLocoFunction(word address, byte function)
{
	byte power;
	if (getLocoFunction(address, function, &power))
	{
		return setLocoFunction(address, function, power ? 0 : 1);
	}

	return false;
}

boolean TrackController::setAccessory(word address, byte position, byte power,
									  word time)
{
	TrackMessage message;

	message.clear();
	message.command = 0x0b;
	message.length = 0x06;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);
	message.data[4] = position;
	message.data[5] = power;

	exchangeMessage(message, message, 1000);

	if (time != 0)
	{
		delay(time);

		message.clear();
		message.command = 0x0b;
		message.length = 0x06;
		message.data[2] = highByte(address);
		message.data[3] = lowByte(address);
		message.data[4] = position;

		exchangeMessage(message, message, 1000);
	}

	return true;
}

boolean TrackController::setAccessory2(word address, byte position, byte power,
									   word time)
{
	TrackMessage message;

	message.clear();
	message.command = 0x0b;
	message.length = 0x06;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);
	message.data[4] = position;
	message.data[5] = power;

	sendMessage(message);

	return true;
}

boolean TrackController::setTurnout(word address, boolean straight)
{
	return setAccessory(address, straight ? ACC_STRAIGHT : ACC_ROUND, 1, 0000);
}

boolean TrackController::getLocoDirection(word address, byte *direction)
{
	TrackMessage message;

	message.clear();
	message.command = 0x05;
	message.length = 0x04;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);

	if (exchangeMessage(message, message, 1000))
	{
		direction[0] = message.data[4];
		return true;
	}
	else
	{
		return false;
	}
}

boolean TrackController::getLocoSpeed(word address, word *speed)
{
	TrackMessage message;

	message.clear();
	message.command = 0x04;
	message.length = 0x04;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);

	if (exchangeMessage(message, message, 1000))
	{
		speed[0] = word(message.data[4], message.data[5]);
		return true;
	}
	else
	{
		return false;
	}
}

boolean TrackController::getLocoFunction(word address, byte function,
										 byte *power)
{
	TrackMessage message;

	message.clear();
	message.command = 0x06;
	message.length = 0x05;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);
	message.data[4] = function;

	if (exchangeMessage(message, message, 1000))
	{
		power[0] = message.data[5];
		return true;
	}
	else
	{
		return false;
	}
}

boolean TrackController::getAccessory(word address, byte *position, byte *power)
{
	TrackMessage message;

	message.clear();
	message.command = 0x0b;
	message.length = 0x04;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);

	if (exchangeMessage(message, message, 1000))
	{
		position[0] = message.data[4];
		power[0] = message.data[5];
		return true;
	}
	else
	{
		return false;
	}
}

boolean TrackController::getAccessory2(word address)
{
	TrackMessage message;

	message.clear();
	message.command = 0x0b;
	message.length = 0x04;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);

	return sendMessage(message);
}

boolean TrackController::writeConfig(word address, word number, byte value)
{
	TrackMessage message;

	message.clear();
	message.command = 0x08;
	message.length = 0x08;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);
	message.data[4] = highByte(number);
	message.data[5] = lowByte(number);
	message.data[6] = value;

	return exchangeMessage(message, message, 10000);
}

boolean TrackController::readConfig(word address, word number, byte *value)
{
	TrackMessage message;

	message.clear();
	message.command = 0x07;
	message.length = 0x07;
	message.data[2] = highByte(address);
	message.data[3] = lowByte(address);
	message.data[4] = highByte(number);
	message.data[5] = lowByte(number);
	message.data[6] = 0x01;

	if (exchangeMessage(message, message, 10000))
	{
		value[0] = message.data[6];
		return true;
	}
	else
	{
		return false;
	}
}

boolean TrackController::getVersion(byte *high, byte *low)
{
	boolean result = false;

	TrackMessage message;

	message.clear();
	message.command = 0x18;

	sendMessage(message);

	delay(500);

	while (receiveMessage(message))
	{
		if (message.command = 0x18 && message.data[6] == 0x00 && message.data[7] == 0x10)
		{
			(*high) = message.data[4];
			(*low) = message.data[5];
			result = true;
		}
	}

	return result;
}

boolean TrackController::getSystemStatus(uint32_t uid, byte channel, word *status)
{
	TrackMessage message;

	message.clear();
	message.command = 0x00;
	message.length = 0x06;
	message.data[0] = uid >> 24;
	message.data[1] = uid >> 16;
	message.data[2] = uid >> 8;
	message.data[3] = uid;
	message.data[4] = 0x0b; // Status
	message.data[5] = channel;

	if (!exchangeMessage(message, message, 1000))
		return false;

	if (message.length != 8)
		return false;

	*status = word(message.data[6], message.data[7]);

	return true;
}
