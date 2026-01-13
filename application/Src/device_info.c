/**
  ******************************************************************************
  * @file           : device_info.c
  * @brief          : Implementing persistent device identification storage that
  *                   can be programmed in developer mode to store serial number,
  *                   model, and manufacturing date information.
  * @project        : Invictus HOTAS Firmware
  * @author         : Invictus Cockpit Systems
  * @version        : 1.1.1
  * @date           : 2026-01-02
  *
  * This file incorporates code from FreeJoy by Yury Vostrenkov (2020)
  * https://github.com/FreeJoy-Team/FreeJoy
  *
  * Licensed under the GNU General Public License v3.0 or later.
  * https://www.gnu.org/licenses/gpl-3.0.html
  *
  * © 2025 Invictus Cockpit Systems. All modifications reserved.
  * This firmware is designed exclusively for Invictus HOTAS hardware.
  *
  ******************************************************************************
  */

#include <string.h>
#include <stdbool.h>  
#include "device_info.h"
#include "common_defines.h"
#include "stm32f10x_flash.h"
#include "common_types.h"




// Global device info in RAM
device_info_t g_device_info = {
    .magic = DEVICE_INFO_MAGIC,
    .version = 1,
    .locked = 0,
    .crc32 = 0,
    .model_number = "",
    .serial_number = "",
    .manufacture_date = "",
    .device_name = "",
    .adc_pga = {8, 8, 8, 8},  // Default: All channels PGA8 (±0.512V for differential mode)
    .adc_mode = {1, 1, 1, 1}  // Default: all differential mode
};

// CRC32 calculation (same as force anchors)
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

/**
 * @brief Reads device info from flash
 * @return 1 if valid device info found, 0 otherwise
 */
static uint8_t device_info_read_flash(void)
{
    const device_info_t *flash = (const device_info_t*)DEVICE_INFO_ADDR;
    
    // Check magic and version
    if (flash->magic != DEVICE_INFO_MAGIC || flash->version != 1) {
        return 0;
    }
    
    // Verify CRC
    device_info_t tmp;
    memcpy(&tmp, flash, sizeof(tmp));
    uint32_t saved_crc = tmp.crc32;
    tmp.crc32 = 0;
    uint32_t calc_crc = crc32_le(&tmp, sizeof(tmp));
    
    if (calc_crc != saved_crc) {
        return 0;
    }
    
    // Copy to RAM
    memcpy(&g_device_info, flash, sizeof(g_device_info));
    return 1;
}

/**
 * @brief Writes device info to flash
 * @return 1 on success, 0 on failure
 */
uint8_t device_info_write_flash(void)
{
    // Don't write if locked
    if (g_device_info.locked) {
        return 0;
    }
			
		// IMPORTANT: Set the header fields before writing
    g_device_info.magic = DEVICE_INFO_MAGIC;
    g_device_info.version = 1;
    g_device_info.locked = 0;
    
    // Update CRC
    g_device_info.crc32 = 0;
    g_device_info.crc32 = crc32_le(&g_device_info, sizeof(g_device_info));
		

    
    // Write to flash
    // IMPORTANT: DEVICE_INFO shares a page with FACTORY (force anchors at FACTORY_ADDR).
    // We must preserve the force anchors when writing device info!
    // Strategy: Read entire page, modify device info section, erase page, write back.

    uint8_t page_buffer[FLASH_PAGE_SIZE];

    // Read entire FACTORY page into buffer
    const uint8_t *page_start = (const uint8_t*)FACTORY_ADDR;
    for (uint16_t i = 0; i < FLASH_PAGE_SIZE; i++) {
        page_buffer[i] = page_start[i];
    }

    // Modify the device info section in the buffer
    const uint32_t offset = DEVICE_INFO_ADDR - FACTORY_ADDR;
    memcpy(&page_buffer[offset], &g_device_info, sizeof(g_device_info));

    // Erase and rewrite the entire page
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if (FLASH_ErasePage(FACTORY_ADDR) != FLASH_COMPLETE) {
        FLASH_Lock();
        return 0;
    }

    // Write back entire page as halfwords
    const uint16_t *hw_buf = (const uint16_t*)page_buffer;
    const uint32_t total_halfwords = FLASH_PAGE_SIZE / 2u;

    for (uint32_t i = 0; i < total_halfwords; ++i) {
        if (FLASH_ProgramHalfWord(FACTORY_ADDR + (i * 2u), hw_buf[i]) != FLASH_COMPLETE) {
            FLASH_Lock();
            return 0;
        }
    }

    FLASH_Lock();
    return 1;
}

/**
 * @brief Initialize device info (called at startup)
 */
void device_info_init(void)
{
    const device_info_t *flash = (const device_info_t*)DEVICE_INFO_ADDR;
    
    // Debug: check first few bytes
    const uint8_t *bytes = (const uint8_t*)flash;
    // You could set a breakpoint here or use your debug method to see these values
    
    if (!device_info_read_flash()) {
        // Initialize with defaults if not found
        memset(&g_device_info, 0, sizeof(g_device_info));
        g_device_info.magic = DEVICE_INFO_MAGIC;
        g_device_info.version = 1;
        g_device_info.locked = 0;
        strcpy(g_device_info.model_number, "UNDEFINED");
        strcpy(g_device_info.serial_number, "000000");
        strcpy(g_device_info.device_name, "Invictus SSC");
        g_device_info.adc_pga[0] = 8;  // Channel 0: PGA 8 (±0.512V for differential)
        g_device_info.adc_pga[1] = 8;  // Channel 1: PGA 8 (±0.512V for differential)
        g_device_info.adc_pga[2] = 8;  // Channel 2: PGA 8 (±0.512V for differential)
        g_device_info.adc_pga[3] = 8;  // Channel 3: PGA 8 (±0.512V for differential)
        g_device_info.adc_mode[0] = 1; // Channel 0: Differential
        g_device_info.adc_mode[1] = 1; // Channel 1: Differential
        g_device_info.adc_mode[2] = 1; // Channel 2: Differential
        g_device_info.adc_mode[3] = 1; // Channel 3: Differential
    }
}


/**
 * @brief Handle device info operations from USB
 */
bool device_info_handle_op(uint8_t op,
                          const uint8_t *in_payload, uint8_t in_len,
                          uint8_t *out_buf, uint8_t *out_len)
{
    if (!out_buf || !out_len) return false;
    *out_len = 0;
    switch (op)
    {
    case OP_GET_DEVICE_INFO: {
        device_info_t blk;
        if (!device_info_read_flash()) {
            // If not initialized/invalid, return defaults so GUI can warn.
            memset(&blk, 0, sizeof(blk));
            blk.magic = DEVICE_INFO_MAGIC;
            blk.version = 1;
            strcpy(blk.model_number, "UNDEFINED");
            strcpy(blk.serial_number, "000000");
            strcpy(blk.device_name, "Invictus SSC");
            blk.adc_pga[0] = 8;  // Default PGA values - all PGA8 for differential
            blk.adc_pga[1] = 8;
            blk.adc_pga[2] = 8;
            blk.adc_pga[3] = 8;
            blk.adc_mode[0] = 1; // Default to differential mode
            blk.adc_mode[1] = 1;
            blk.adc_mode[2] = 1;
            blk.adc_mode[3] = 1;
        } else {
            // Copy from global to struct
            memset(&blk, 0, sizeof(blk));
            blk.magic = DEVICE_INFO_MAGIC;
            blk.version = 1;
            blk.locked = g_device_info.locked;
            memcpy(blk.model_number, g_device_info.model_number, sizeof(blk.model_number));
            memcpy(blk.serial_number, g_device_info.serial_number, sizeof(blk.serial_number));
            memcpy(blk.manufacture_date, g_device_info.manufacture_date, sizeof(blk.manufacture_date));
            memcpy(blk.device_name, g_device_info.device_name, sizeof(blk.device_name));
            memcpy(blk.adc_pga, g_device_info.adc_pga, sizeof(blk.adc_pga));
            memcpy(blk.adc_mode, g_device_info.adc_mode, sizeof(blk.adc_mode));
        }
        memcpy(out_buf, &blk, sizeof(blk));
        *out_len = (uint8_t)sizeof(blk);
        return true;
    }
    case OP_SET_DEVICE_INFO: {
        if (!in_payload || in_len != sizeof(device_info_t)) {
            return false; // bad length ? let caller decide how to NAK
        }
        const device_info_t *in = (const device_info_t*)in_payload;
        // Copy to global
        memcpy(g_device_info.model_number, in->model_number, sizeof(g_device_info.model_number));
        memcpy(g_device_info.serial_number, in->serial_number, sizeof(g_device_info.serial_number));
        memcpy(g_device_info.manufacture_date, in->manufacture_date, sizeof(g_device_info.manufacture_date));
        memcpy(g_device_info.device_name, in->device_name, sizeof(g_device_info.device_name));
        memcpy(g_device_info.adc_pga, in->adc_pga, sizeof(g_device_info.adc_pga));
        memcpy(g_device_info.adc_mode, in->adc_mode, sizeof(g_device_info.adc_mode));

        bool ok = device_info_write_flash();
        out_buf[0] = ok ? 1 : 0;   // 1=OK, 0=FAIL (e.g., locked)
        *out_len = 1;
        return true;
    }
    
    }
}