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

#ifndef RailuinoSeeed__h
#define RailuinoSeeed__h

#include <Arduino.h>
#include <Printable.h>

// ===================================================================
// === Board detection ===============================================
// ===================================================================

#if defined(__AVR_ATmega328P__)
#define __UNO__ 1
#define __BOARD__ "Arduino Uno"
#elif defined(__AVR_ATmega32U4__)
#define __LEONARDO__ 1
#define __BOARD__ "Arduino Leonardo"
#else
#error Unsupported board. Please adjust library.
#endif

// ===================================================================
// === Common definitions ============================================
// ===================================================================

/**
 * Version of Railuino library and required connection box software.
 */
#define RAILUINO_VERSION 0x005A // 0.90
#define TRACKBOX_VERSION 0x0127 // 1.39

/**
 * Constants for protocol base addresses.
 */
#define ADDR_MM2 0x0000     // MM2 locomotive
#define ADDR_SX1 0x0800     // Selectrix (old) locomotive
#define ADDR_MFX 0x4000     // MFX locomotive
#define ADDR_SX2 0x8000     // Selectrix (new) locomotive
#define ADDR_DCC 0xC000     // DCC locomotive
#define ADDR_ACC_SX1 0x2000 // Selectrix (old) magnetic accessory
#define ADDR_ACC_MM2 0x2FFF // MM2 magnetic accessory
#define ADDR_ACC_DCC 0x3800 // DCC magnetic accessory

/**
 * Constants for classic MM2 Delta addresses.
 */
#define DELTA1 78
#define DELTA2 72
#define DELTA3 60
#define DELTA4 24

/**
 * Constants for locomotive directions.
 */
#define DIR_CURRENT 0
#define DIR_FORWARD 1
#define DIR_REVERSE 2
#define DIR_CHANGE 3

/**
 * Constants for accessory states including some aliases.
 */
#define ACC_OFF 0
#define ACC_ROUND 0
#define ACC_RED 0
#define ACC_RIGHT 0
#define ACC_HP0 0

#define ACC_ON 1
#define ACC_GREEN 1
#define ACC_STRAIGHT 1
#define ACC_HP1 1

#define ACC_YELLOW 2
#define ACC_LEFT 2
#define ACC_HP2 2

#define ACC_WHITE 3
#define ACC_SH0 3

/**
 * Represents a message going through the Marklin CAN bus. More or
 * less a beautified version of the real CAN message. You normally
 * don't need to use this unless you want to experiment with the
 * protocol or extend the library. See the Marklin protocol
 * documentation for details. The TrackMessage is a Printable, so
 * it can be directly used in Serial.println(), for instance. It
 * can also be converted from a String.
 */
class TrackMessage : public Printable
{

public:
  /**
   * The command number.
   */
  byte command;

  /**
   * The hash that is used for avoiding device/message collisions.
   */
  word hash;

  /**
   * Whether this is a response to a request.
   */
  boolean response;

  /**
   * The number of data bytes in the payload.
   */
  byte length;

  /**
   * The actual message data bytes.
   */
  byte data[8];

  /**
   * Clears the message, setting all values to zero. Provides for
   * easy recycling of TrackMessage objects.
   */
  void clear();

  /**
   * Prints the message to the given Print object, which could be a
   * Serial object, for instance. The message format looks like this
   *
   * HHHH R CC L DD DD DD DD DD DD DD DD
   *
   * with all numbers being hexadecimals and the data bytes being
   * optional beyond what the message length specifies. Exactly one
   * whitespace is inserted between different fields as a separator.
   */
  virtual size_t printTo(Print &p) const;

  /**
   * Parses the message from the given String. Returns true on
   * success, false otherwise. The message must have exactly the
   * format that printTo creates. This includes each and every
   * whitespace. If the parsing fails the state of the object is
   * undefined afterwards, and a clear() is recommended.
   */
  boolean parseFrom(String &s);

  /**
   * Parses the message from the CAN message data returned by
   * MCP_CAN::readMsgBufID().
   */
  boolean fromCanMsg(unsigned long aId, byte aExt, byte aRtr, byte aLen, byte *aBuf);
};

class MCP_CAN;

class TrackController
{
public:
  TrackController(word aHash, boolean aDebug = false)
      : mHash(aHash), mDebug(aDebug)
  {
  }

  /**
   * Initialises the TrackController with the MCP_CAN object used
   * for communication over the CAN-Bus Shield.
   */
  void init(MCP_CAN &aCAN);

  /**
   * Sends a message and reports true on success. Internal method.
   * Normally you don't want to use this, but the more convenient
   * methods below instead.
   */
  boolean sendMessage(TrackMessage &message);

  /**
   * Receives an arbitrary message, if available, and reports true
   * on success. Does not block. Internal method. Normally you
   * don't want to use this, but the more convenient methods below
   * instead.
   */
  boolean receiveMessage(TrackMessage &message);

  /**
   * Sends a message and waits for the corresponding response,
   * returning true on success. Blocks until either a message with
   * the same command ID and the response marker arrives or the
   * timeout (in ms) expires. All non-matching messages are
   * skipped. Internal method. Normally you don't want to use this,
   * but the more convenient methods below instead. 'out' and 'in'
   * may be the same object.
   */
  boolean exchangeMessage(TrackMessage &out, TrackMessage &in, word timeout);

  /**
   * Controls power on the track. When passing false, all
   * locomotives will stop, but remember their previous directions
   * and speeds. When passing true, all locomotives will regain
   * their old directions and speeds. The system starts in
   * stopped mode in order to avoid accidents. The return value
   * reflects whether the call was successful.
   */
  boolean setPower(boolean power);
  boolean setPower2(boolean power);
  boolean getPower2(void);

  /**
   * Sets the direction of the given locomotive. Valid directions
   * are those specified by the DIR_* constants. The return value
   * reflects whether the call was successful.
   */
  boolean setLocoDirection(word address, byte direction);

  /**
   * Toggles the direction of the given locomotive. This normally
   * includes a full stop.
   */
  boolean toggleLocoDirection(word address);

  /**
   * Sets the speed of the given locomotive. Valid speeds are
   * 0 to 1023 (inclusive), though the connector box will limit
   * all speeds beyond 1000 to 1000. The return value reflects
   * whether the call was successful.
   */
  boolean setLocoSpeed(word address, word speed);

  /**
   * Increases the speed of the given locomotive by 1/14th
   * of the maximum speed.
   */
  boolean accelerateLoco(word address);

  /**
   * Decreases the speed of the given locomotive by 1/14th
   * of the maximum speed.
   */
  boolean decelerateLoco(word address);

  /**
   * Sets the given function of the given locomotive (or simply a
   * function decoder). Valid functions are 0 to 31, with 0
   * normally denoting the head/backlight. Valid values are, again,
   * 0 ("off") to 31, although not all protocols support values
   * beyond 1 (which then means "on").  The return value reflects
   * whether the call was successful.
   */
  boolean setLocoFunction(word address, byte function, byte power);

  /**
   * Toggles the given function of the given locomotive. Valid
   * functions are 0 to 31, with 0 normally denoting the
   * head/backlight.
   */
  boolean toggleLocoFunction(word address, byte function);

  /**
   * Switches the given magnetic accessory. Valid position values
   * are those denoted by the ACC_* constants. Valid power values
   * are 0 ("off") to 31 (inclusive) although not all protocols
   * support values beyond 1 (which then means "on"). The final
   * parameter specifies the time (in ms) for which the accessory
   * will be active. A time of 0 means the accessory will only be
   * switched on. Some magnetic accessories must not be active for
   * too long, because they might burn out. A good timeout for
   * Marklin turnouts seems to be 20 ms. The return value reflects
   * whether the call was successful.
   */
  boolean setAccessory(word address, byte position, byte power, word time);

  boolean setAccessory2(word address, byte position, byte power, word time);

  /**
   * Switches a turnout. This is actually a convenience function
   * around setAccessory() that uses default values for some
   * parameters. The return value reflects whether the call was
   * successful.
   */
  boolean setTurnout(word address, boolean straight);

  /**
   * Queries the direction of the given locomotive and writes it
   * into the referenced byte. The return value indicates whether
   * the call was successful and the direction is valid.
   */
  boolean getLocoDirection(word address, byte *direction);

  /**
   * Queries the speed of the given locomotive and writes it
   * into the referenced byte. The return value indicates whether
   * the call was successful and the speed is valid.
   */
  boolean getLocoSpeed(word address, word *speed);

  /**
   * Queries the given function of the given locomotive and writes
   * it into the referenced byte. The return value indicates
   * whether the call was successful and the power is valid. Note
   * that the call will not reflect the original power value sent
   * to the function, but only 0 ("off") or 1 ("on").
   */
  boolean getLocoFunction(word address, byte function, byte *power);

  /**
   * Queries the given magnetic accessory's state and and writes
   * it into the referenced bytes. The return value indicates
   * whether the call was successful and the bytes are valid. Note
   * that the call will not reflect the original power value sent
   * to the function, but only 0 ("off") or 1 ("on").
   */
  boolean getAccessory(word address, byte *position, byte *power);
  boolean getAccessory2(word address);

  /**
   * Queries a turnout state. This is actually a convenience
   * function around getAccessory() that uses default values for
   * some parameters.
   */
  boolean getTurnout(word address, boolean *straight);

  /**
   * Writes the given value to the given config number of the given
   * locomotive.  The return value reflects whether the call was
   * successful.
   */
  boolean writeConfig(word address, word number, byte value);

  /**
   * Reads the given config number of the given locomotive into the
   * given value.
   */
  boolean readConfig(word address, word number, byte *value);

  /**
   * Queries the software version of the track format processor.
   */
  boolean getVersion(byte *high, byte *low);

  /**
   * Queries the system status of the track format processor ("Gleis Format
   * Prozessor") with the id "uid" for the given channel. The uid 0 can be used
   * if only one track format processor is connected.
   */
  boolean getSystemStatus(uint32_t uid, byte channel, word *status);

private:
  MCP_CAN *mCAN = nullptr;
  word mHash = 0;
  boolean mDebug = false;
};

#endif
