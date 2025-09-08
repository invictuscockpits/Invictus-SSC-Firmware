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
    .manufacture_date = ""
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
static uint8_t device_info_write_flash(void)
{
    // Don't write if locked
    if (g_device_info.locked) {
        return 0;
    }
    
    // Update CRC
    g_device_info.crc32 = 0;
    g_device_info.crc32 = crc32_le(&g_device_info, sizeof(g_device_info));
    
    // Write to flash
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    
    if (FLASH_ErasePage(DEVICE_INFO_ADDR) != FLASH_COMPLETE) {
        FLASH_Lock();
        return 0;
    }
    
    const uint16_t *hw = (const uint16_t*)&g_device_info;
    const uint32_t halfwords = (sizeof(g_device_info) + 1u) / 2u;
    
    for (uint32_t i = 0; i < halfwords; ++i) {
        if (FLASH_ProgramHalfWord(DEVICE_INFO_ADDR + (i * 2u), hw[i]) != FLASH_COMPLETE) {
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
    }
}


/**
 * @brief Handle device info operations from USB
 */
bool device_info_handle_op(uint8_t op, const uint8_t *in_payload, uint8_t in_len,
                          uint8_t *out_buf, uint8_t *out_len)
{
    if (!out_buf || !out_len) return false;
    *out_len = 0;

    switch (op)
    {
    case OP_GET_DEVICE_INFO: {
        memcpy(out_buf, &g_device_info, sizeof(g_device_info));
        *out_len = sizeof(g_device_info);
        return true;
    }

    case OP_SET_DEVICE_INFO: {
    if (!in_payload || in_len < sizeof(device_info_t)) {
        return false;
    }
    
    // Debug: return the address we're writing to
    out_buf[0] = 1;  // Success
    out_buf[1] = (DEVICE_INFO_ADDR >> 16) & 0xFF;  // Address high byte
    out_buf[2] = (DEVICE_INFO_ADDR >> 8) & 0xFF;   // Address mid byte  
    out_buf[3] = DEVICE_INFO_ADDR & 0xFF;          // Address low byte
    *out_len = 4;
    
    // Do the actual write...
    const device_info_t *in = (const device_info_t*)in_payload;
    memcpy(g_device_info.model_number, in->model_number, sizeof(g_device_info.model_number));
    memcpy(g_device_info.serial_number, in->serial_number, sizeof(g_device_info.serial_number));
    memcpy(g_device_info.manufacture_date, in->manufacture_date, sizeof(g_device_info.manufacture_date));
    
    uint8_t ok = device_info_write_flash();
    out_buf[0] = ok ? 1 : 0;
    return true;
}
    
    default:
        return false;  // Add explicit default case
    }
}
