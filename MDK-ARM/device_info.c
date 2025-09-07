/**
  ******************************************************************************
  * @file           : device_info.c
  * @brief          : Implementing persistent device identification storage that 
  *                   can be programmed in developer mode to store serial number,
  *                   model, and manufacturing date information.
  * @project        : Invictus HOTAS Firmware
  * @author         : Invictus Cockpit Systems
  * @version        : 1.0.0
  * @date           : 2025-09-07
  *
  * Based on FreeJoy firmware by Yury Vostrenkov (2020)
  * https://github.com/FreeJoy-Team/FreeJoy
  *
  * This software includes original or modified portions of FreeJoy, distributed
  * under the terms of the GNU General Public License v3.0 or later:
  * https://www.gnu.org/licenses/gpl-3.0.html
  *
  * Modifications and additions are © 2025 Invictus Cockpit Systems.
  *
  * This software has been carefully modified for a specific purpose. It is not 
  * recommended for use outside of the Invictus HOTAS system.
  *
  ******************************************************************************
  */

#include <string.h>
#include "device_info.h"
#include "common_defines.h"
#include "stm32f10x_flash.h"

#define DEVICE_INFO_ADDR (FACTORY_ADDR - FLASH_PAGE_SIZE)  // One page before factory
#define DEVICE_INFO_MAGIC 0xDEF0
#define DEVICE_INFO_VERSION 0x01

// Pointer to flash-resident device info
static inline const device_info_t* DI_ptr(void) {
    return (const device_info_t*)DEVICE_INFO_ADDR;
}

// CRC32 implementation (same as force anchors)
static uint32_t crc32_le(const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t*)data;
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        crc ^= *p++;
        for (int k = 0; k < 8; ++k)
            crc = (crc >> 1) ^ (0xEDB88320u & (-(int32_t)(crc & 1)));
    }
    return ~crc;
}

// Flash write helper
static bool flash_write_device_info(const device_info_t *src)
{
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if (FLASH_ErasePage(DEVICE_INFO_ADDR) != FLASH_COMPLETE) {
        FLASH_Lock();
        return false;
    }

    const uint16_t *hw = (const uint16_t*)src;
    const uint32_t halfwords = (sizeof(*src) + 1u) / 2u;

    for (uint32_t i = 0; i < halfwords; ++i) {
        if (FLASH_ProgramHalfWord(DEVICE_INFO_ADDR + (i * 2u), hw[i]) != FLASH_COMPLETE) {
            FLASH_Lock();
            return false;
        }
    }

    FLASH_Lock();
    return true;
}

// Public API
bool device_info_valid(const device_info_t *blk)
{
    if (!blk) return false;
    if (blk->magic != DEVICE_INFO_MAGIC) return false;
    if (blk->version != DEVICE_INFO_VERSION) return false;

    device_info_t tmp;
    memcpy(&tmp, blk, sizeof(tmp));
    uint32_t saved = tmp.crc32;
    tmp.crc32 = 0;
    return (crc32_le(&tmp, sizeof(tmp)) == saved);
}

bool device_info_read(device_info_t *out)
{
    if (!out) return false;
    const device_info_t *flash = DI_ptr();
    if (!device_info_valid(flash)) return false;
    memcpy(out, flash, sizeof(*out));
    return true;
}

bool device_info_write(const device_info_t *in)
{
    if (!in) return false;

    // Check if already locked
    const device_info_t *flash = DI_ptr();
    if (device_info_valid(flash) && flash->locked) return false;

    // Prepare data with normalized header
    device_info_t w;
    memcpy(&w, in, sizeof(w));
    w.magic = DEVICE_INFO_MAGIC;
    w.version = DEVICE_INFO_VERSION;
    w.locked = 0;
    w.crc32 = 0;
    w.crc32 = crc32_le(&w, sizeof(w));

    if (!flash_write_device_info(&w)) return false;
    return device_info_valid(DI_ptr());
}

bool device_info_lock(void)
{
    const device_info_t *flash = DI_ptr();
    if (!device_info_valid(flash)) return false;
    if (flash->locked) return true;

    device_info_t w;
    memcpy(&w, flash, sizeof(w));
    w.locked = 1;
    w.crc32 = 0;
    w.crc32 = crc32_le(&w, sizeof(w));

    if (!flash_write_device_info(&w)) return false;
    return device_info_valid(DI_ptr()) && DI_ptr()->locked == 1;
}

// Handler for HID operations
bool device_info_handle_op(uint8_t op, const uint8_t *in_payload, uint8_t in_len,
                          uint8_t *out_buf, uint8_t *out_len)
{
    if (!out_buf || !out_len) return false;
    *out_len = 0;

    switch (op)
    {
    case OP_GET_DEVICE_INFO: {
        device_info_t info;
        if (!device_info_read(&info)) {
            memset(&info, 0, sizeof(info));
        }
        memcpy(out_buf, &info, sizeof(info));
        *out_len = (uint8_t)sizeof(info);
        return true;
    }

    case OP_SET_DEVICE_INFO: {
        if (!in_payload || in_len != sizeof(device_info_t)) {
            return false;
        }
        const device_info_t *in = (const device_info_t*)in_payload;
        bool ok = device_info_write(in);
        out_buf[0] = ok ? 1 : 0;
        *out_len = 1;
        return true;
    }

    default:
        return false;
    }
}