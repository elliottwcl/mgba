/* Copyright (c) 2013-2021 Jeffrey Pfau
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#endif

#include <mgba/core/config.h>
#include <mgba/internal/gba/io.h>
#include <mgba/internal/gba/serialize.h>
#include <mgba/internal/gba/cart/chis.h>
#include <mgba/internal/gba/cart/gpio.h>



uint64_t _get_current_timestamp_milliseconds() {
    uint64_t timestamp = 0;
#ifdef _WIN32
    FILETIME ft;
    GetSystemTimeAsFileTime(&ft);
    static const uint64_t EPOCH = ((uint64_t)116444736000000000ULL);
    uint64_t time = ((uint64_t)ft.dwLowDateTime | ((uint64_t)ft.dwHighDateTime << 32)) / 10;
    timestamp = (time - EPOCH) / 10000;
#else
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) == 0) {
        timestamp = (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
    } else {
        perror("clock_gettime failed");
        exit(EXIT_FAILURE);
    }
#endif
    return timestamp;
}

void _sleep_cross_platform(unsigned int milliseconds) {
#ifdef _WIN32
    Sleep(milliseconds);
#else
    usleep(milliseconds * 1000);
#endif
}

void _setRumble(struct ChisCartridgeHardware* hw, bool enable) {
	struct mRumble* rumble = hw->gpio->p->rumble;
	if (!rumble) {
		return;
	}

	int32_t currentTime = mTimingCurrentTime(&hw->gpio->p->timing);
	rumble->setRumble(rumble, enable, currentTime - hw->gpio->p->lastRumble);
	hw->gpio->p->lastRumble = currentTime;
}

#ifndef _WIN32
void* _rumbleOff(void* context) {
#else
DWORD WINAPI _rumbleOff(LPVOID context) {
#endif
    struct ChisCartridgeHardware* hw = (struct ChisCartridgeHardware*)context;
    // double check
    while (!hw->stopThread)
    {
        uint64_t ts = _get_current_timestamp_milliseconds();
        MutexLock(&hw->gpioMutex);
        if (hw->rumbleWaitCommit == 0 && hw->rumble == 1 && hw->lastOffTS != 0 && hw->lastOffTS < ts) {
            _setRumble(hw, 0);
            hw->rumble = 0;
            hw->lastOffTS = 0;
        }
        MutexUnlock(&hw->gpioMutex);
        _sleep_cross_platform(1);
    }
    return NULL;
}

void ChisCartridgeHardwareInit(struct ChisCartridgeHardware* hw, struct GBACartridgeHardware* gpio) {
    hw->gpio = gpio;
    hw->rumbleStatus = EZ_RUMBLE_NONE;
    hw->delayOffThread = NULL;
    MutexInit(&hw->gpioMutex);
    hw->lastOffTS = 0;
    hw->rumbleWaitCommit = -1;
    hw->rumble = -1;
    hw->stopThread = false;
    ThreadCreate(&hw->delayOffThread, _rumbleOff, hw);
    // set rumble gpio on
    GBAHardwareGPIOWrite(hw->gpio, 0xC8, 1);
    GBAHardwareGPIOWrite(hw->gpio, 0xC6, 8);
}

void ChisCartridgeHardwareDeinit(struct ChisCartridgeHardware* hw) {
    hw->gpio = NULL;
    hw->rumbleStatus = EZ_RUMBLE_NONE;
    hw->rumbleWaitCommit = -1;
    hw->rumble = -1;
    hw->stopThread = true;
    ThreadJoin(hw->delayOffThread);
    MutexDeinit(&hw->gpioMutex);
}


void _commitRumble(struct ChisCartridgeHardware* hw) {
    if (hw->rumbleWaitCommit == 1) {
        MutexLock(&hw->gpioMutex);
        hw->lastOffTS = 0;
        _setRumble(hw, 1);
        hw->rumble = 1;
        MutexUnlock(&hw->gpioMutex);
    } else if (hw->rumbleWaitCommit == 0) {
        uint64_t ts = _get_current_timestamp_milliseconds();
        MutexLock(&hw->gpioMutex);
        _setRumble(hw, 1);
        hw->lastOffTS = ts + 400;// 400ms delay ?
        MutexUnlock(&hw->gpioMutex);
    }
}

void ChisCartridgeHardwareWrite32(struct ChisCartridgeHardware* hw, uint32_t address, uint32_t value) {
    value &= 0xFFFF;
    uint16_t value16 = value & 0xFFFF;
    uint8_t value8 = value & 0xFF;
    switch (hw->rumbleStatus) {
        case EZ_RUMBLE_NONE:
            if (address == 0x09FE0000 && value16 == 0xD200) {
                hw->rumbleStatus = EZ_RUMBLE_START_CMD_1;
            } else if (address == 0x09E20000 && value8 == 0x8) {
                hw->rumbleWaitCommit = 0;
                hw->rumbleStatus = EZ_RUMBLE_DATA_5;
            } else if (address == 0x08001000 && value8 == 0) {
                hw->rumbleWaitCommit = 0;
                _commitRumble(hw);
                hw->rumbleStatus = EZ_RUMBLE_NONE;
            } else {
                hw->rumbleStatus = EZ_RUMBLE_NONE;
            }
            break;
        case EZ_RUMBLE_START_CMD_1:
            if (address == 0x08000000 && value16 == 0x1500) {
                hw->rumbleStatus = EZ_RUMBLE_START_CMD_2;
            } else {
                hw->rumbleStatus = EZ_RUMBLE_NONE;
            }
            break;
        case EZ_RUMBLE_START_CMD_2:
            if (address == 0x08020000 && value16 == 0xD200) {
                hw->rumbleStatus = EZ_RUMBLE_START_CMD_3;
            } else {
                hw->rumbleStatus = EZ_RUMBLE_NONE;
            }
            break;
        case EZ_RUMBLE_START_CMD_3:
            if (address == 0x08040000 && value16 == 0x1500) {
                hw->rumbleStatus = EZ_RUMBLE_START_CMD_4;
            } else {
                hw->rumbleStatus = EZ_RUMBLE_NONE;
            }
            break;
        case EZ_RUMBLE_START_CMD_4:
            if (address == 0x09E20000) {
                if (value8 == 0xF1) {
                    hw->rumbleStatus = EZ_RUMBLE_DATA_5;
                } else if (value8 == 7) {
                    hw->rumbleWaitCommit = 1;
                    hw->rumbleStatus = EZ_RUMBLE_DATA_5;
                } else if (value8 == 8) {
                    hw->rumbleWaitCommit = 0;
                    hw->rumbleStatus = EZ_RUMBLE_DATA_5;
                } else {
                    hw->rumbleStatus = EZ_RUMBLE_NONE;
                }
            } else {
                hw->rumbleStatus = EZ_RUMBLE_NONE;
            }
            break;
        case EZ_RUMBLE_DATA_5:
        case EZ_RUMBLE_DATA_5_3IN1:
            if (address == 0x09FC0000 && value16 == 0x1500) {
                //commit ez3in1 rumble
                if (hw->rumbleStatus == EZ_RUMBLE_DATA_5_3IN1) {
                    _commitRumble(hw);
                    hw->rumbleStatus = EZ_RUMBLE_NONE;
                } else {
                    hw->rumbleStatus = EZ_RUMBLE_END_CMD_6;
                }
            } else {
                hw->rumbleStatus = EZ_RUMBLE_NONE;
            }
            break;
        case EZ_RUMBLE_END_CMD_6:
            if (address == 0x08001000) {
                if (value8 == 2) {
                    hw->rumbleWaitCommit = 1;
                } else if (value8 == 0) {
                    hw->rumbleWaitCommit = 0;
                } else {
                    hw->rumbleWaitCommit = 0;
                }
                // commit ezode rumble
                _commitRumble(hw);
                hw->rumbleStatus = EZ_RUMBLE_NONE;
            } else {
                hw->rumbleStatus = EZ_RUMBLE_NONE;
            }
            break;
        default:
            hw->rumbleStatus = EZ_RUMBLE_NONE;
            break;
    }
}
void ChisCartridgeHardwareWrite16(struct ChisCartridgeHardware* hw, uint32_t address, uint16_t value) {
    ChisCartridgeHardwareWrite32(hw, address, value);
}
void ChisCartridgeHardwareWrite8(struct ChisCartridgeHardware* hw, uint32_t address, uint8_t value) {
    ChisCartridgeHardwareWrite32(hw, address, value);
}