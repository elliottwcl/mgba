/* Copyright (c) 2013-2015 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef EZ_HARDWARE_H
#define EZ_HARDWARE_H

#include <mgba-util/common.h>

CXX_GUARD_START

#include <mgba/core/log.h>
#include <mgba/core/timing.h>
#include <mgba/gba/interface.h>
#include <mgba-util/threading.h>


enum EZRumbleStatus {
	EZ_RUMBLE_NONE,
	EZ_RUMBLE_START_CMD_1, //0xD200
	EZ_RUMBLE_START_CMD_2, //0x1500
	EZ_RUMBLE_START_CMD_3, //0xD200
	EZ_RUMBLE_START_CMD_4, //0x1500
	EZ_RUMBLE_DATA_5,      // 0xF1 for ezode
	EZ_RUMBLE_DATA_5_3IN1, // 7 for ez3in1 rumble ON, 8 for ez3in1 rumble OFF
	EZ_RUMBLE_END_CMD_6,   //0x1500, end of ez3in1 rumble
	EZ_RUMBLE_DATA_7,      // 2 for ezode rumble ON
};


struct ChisCartridgeHardware {
	enum EZRumbleStatus rumbleStatus;
	int rumble;
	int rumbleWaitCommit;
	uint64_t lastOffTS;
	Thread delayOffThread;
	Mutex  gpioMutex;
	bool stopThread;
	struct GBACartridgeHardware* gpio;
};

void ChisCartridgeHardwareInit(struct ChisCartridgeHardware* hw, struct GBACartridgeHardware* gpio);
void ChisCartridgeHardwareDeinit(struct ChisCartridgeHardware* hw);

void ChisCartridgeHardwareWrite32(struct ChisCartridgeHardware* hw, uint32_t address, uint32_t value);
void ChisCartridgeHardwareWrite16(struct ChisCartridgeHardware* hw, uint32_t address, uint16_t value);
void ChisCartridgeHardwareWrite8(struct ChisCartridgeHardware* hw, uint32_t address, uint8_t value);

#define IS_EZ_RUMBLE_ADDRESS(address) (address == 0x09FE0000 || address == 0x08000000 || address == 0x08020000 || address == 0x08040000 || address == 0x09E20000 || address == 0x09FC0000 || address == 0x08001000)

CXX_GUARD_END

#endif
