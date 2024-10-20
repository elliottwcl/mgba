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
#include <mgba/internal/gba/cart/chis.h>
#include <mgba/internal/gba/cart/gpio.h>

mLOG_DEFINE_CATEGORY(GBA_HW, "GBA Pak Hardware", "gba.hardware");

uint64_t _get_current_timestamp_milliseconds() {
    uint64_t timestamp = 0;
#ifdef _WIN32
    timestamp = GetTickCount64();
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
#ifdef _WIN32x
    Sleep(milliseconds);
#else
    usleep(milliseconds * 1000);
#endif
}

void ChisCartridgeHardwareInit(struct ChisCartridgeHardware* hw, struct GBACartridgeHardware* gpio) {
    hw->gpio = gpio;
    hw->rumbleStatus = EZ_RUMBLE_NONE;
    hw->delayOffThread = NULL;
    MutexInit(&hw->gpioMutex);
    hw->lastOffTS = 0;
    hw->rumbleWaitCommit = -1;
}

void ChisCartridgeHardwareDeinit(struct ChisCartridgeHardware* hw) {
    hw->gpio = NULL;
    hw->rumbleStatus = EZ_RUMBLE_NONE;
    hw->rumbleWaitCommit = -1;
    MutexDeinit(&hw->gpioMutex);
}

void* _rumbleOff(void* context) {
    struct ChisCartridgeHardware* hw = (struct ChisCartridgeHardware*)context;
    _sleep_cross_platform(200);
    // double check
    if (hw->rumbleWaitCommit == 0) {
        MutexLock(&hw->gpioMutex);
        GBAHardwareGPIOWrite(hw->gpio, 0xC4, 0);
        MutexUnlock(&hw->gpioMutex);
    }
    return NULL;
}

void _commitRumble(struct ChisCartridgeHardware* hw) {
    uint64_t ts = _get_current_timestamp_milliseconds();
    if (hw->delayOffThread != NULL && hw->lastOffTS != 0 && ts - hw->lastOffTS >= 200) {
        ThreadJoin(hw->delayOffThread);
        hw->delayOffThread = NULL;
        hw->lastOffTS = 0;
    }
    if (hw->rumbleWaitCommit == 1) {
        MutexLock(&hw->gpioMutex);
        GBAHardwareGPIOWrite(hw->gpio, 0xC8, 1);
        GBAHardwareGPIOWrite(hw->gpio, 0xC6, 8);
        GBAHardwareGPIOWrite(hw->gpio, 0xC4, 8);
        MutexUnlock(&hw->gpioMutex);
    } else if (hw->rumbleWaitCommit == 0) {
        if (hw->delayOffThread == NULL) {
            ThreadCreate(&hw->delayOffThread, _rumbleOff, hw);
            hw->lastOffTS = ts;
        }
    }
}

void ChisCartridgeHardwareWrite32(struct ChisCartridgeHardware* hw, uint32_t address, uint32_t value) {
    value &= 0xFFFF;
    uint8_t value8 = value & 0xFF;
    switch (hw->rumbleStatus) {
        case EZ_RUMBLE_NONE:
            if (address == 0x09FE0000 && value == 0xD200) {
                hw->rumbleStatus = EZ_RUMBLE_START_CMD_1;
            } else if (0x09E20000 && value == 0x8) {
                hw->rumbleWaitCommit = 0;
                hw->rumbleStatus = EZ_RUMBLE_DATA_5;
            } else {
                hw->rumbleStatus = EZ_RUMBLE_NONE;
            }
            break;
        case EZ_RUMBLE_START_CMD_1:
            if (address == 0x08000000 && value == 0x1500) {
                hw->rumbleStatus = EZ_RUMBLE_START_CMD_2;
            } else {
                hw->rumbleStatus = EZ_RUMBLE_NONE;
            }
            break;
        case EZ_RUMBLE_START_CMD_2:
            if (address == 0x08020000 && value == 0xD200) {
                hw->rumbleStatus = EZ_RUMBLE_START_CMD_3;
            } else {
                hw->rumbleStatus = EZ_RUMBLE_NONE;
            }
            break;
        case EZ_RUMBLE_START_CMD_3:
            if (address == 0x08040000 && value == 0x1500) {
                hw->rumbleStatus = EZ_RUMBLE_START_CMD_4;
            } else {
                hw->rumbleStatus = EZ_RUMBLE_NONE;
            }
            break;
        case EZ_RUMBLE_START_CMD_4:
            if (address == 0x09E20000) {
                if (value == 0xF1) {
                    hw->rumbleStatus = EZ_RUMBLE_DATA_5;
                } else if (value8 == 7) {
                    hw->rumbleStatus = EZ_RUMBLE_DATA_5;
                } else if (value8 == 8) {
                    hw->rumbleStatus = EZ_RUMBLE_DATA_5;
                }
            } else {
                hw->rumbleStatus = EZ_RUMBLE_NONE;
            }
            break;
        case EZ_RUMBLE_DATA_5:
            if (address == 0x09FC0000 && value == 0x1500) {
                hw->rumbleStatus = EZ_RUMBLE_END_CMD_6;
                // commit ez3in1 rumble
                _commitRumble(hw);
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