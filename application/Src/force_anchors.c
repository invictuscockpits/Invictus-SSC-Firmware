/**
  ******************************************************************************
  * @file           : force_anchors.c
  * @brief          : Implementing persistent force anchors that can be programmed in 
	*										developer mode to prevent users from overwriting device-specific
	*										force values that otherwise cannot be recovered.			
	* @project        Invictus HOTAS Firmware
  * @author         Invictus Cockpit Systems
  * @version        1.1.0
  * @date           2025-09-06
  *
  * Based on FreeJoy firmware by Yury Vostrenkov (2020)
  * https://github.com/FreeJoy-Team/FreeJoy
  *
  * This software includes original or modified portions of FreeJoy, distributed
  * under the terms of the GNU General Public License v3.0 or later:
  * https://www.gnu.org/licenses/gpl-3.0.html
  *
  * Modifications and additions are  2025 Invictus Cockpit Systems.
  *
  * This software has been carefully modified for a specific purpose.  It is not recommended for use outside of the Invictus HOTAS system.
  *

  ******************************************************************************
  */

#include <string.h>
#include "force_anchors.h"
#include "common_defines.h"       // FACTORY_ADDR / FACTORY_MAGIC / FACTORY_VERSION
#include "stm32f10x_flash.h"      // same FLASH API used in config.c


// Pointer to flash-resident page
static inline const force_factory_anchors_t* FA_ptr(void) {
    return (const force_factory_anchors_t*)FACTORY_ADDR;
}

/* ---------------- CRC32 (LE, poly 0xEDB88320) ---------------- */
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

uint32_t force_crc32(const void *data, uint32_t len) { return crc32_le(data, len); }

/* ---------------- Flash write helper (mirror config.c pattern) ---------------- */
static bool flash_write_factory(const force_factory_anchors_t *src)
{
    /* 1) Proper unlock + flag clear */
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    /* 2) Erase and check status */
    if (FLASH_ErasePage(FACTORY_ADDR) != FLASH_COMPLETE) {
        FLASH_Lock();
        return false;
    }

    /* 3) Program as halfwords (more tolerant on F1) */
    const uint16_t *hw = (const uint16_t*)src;
    const uint32_t halfwords = (sizeof(*src) + 1u) / 2u;

    for (uint32_t i = 0; i < halfwords; ++i) {
        if (FLASH_ProgramHalfWord(FACTORY_ADDR + (i * 2u), hw[i]) != FLASH_COMPLETE) {
            FLASH_Lock();
            return false;
        }
    }

    FLASH_Lock();
    return true;
}



/* ---------------- Public API ---------------- */
bool force_anchors_valid(const force_factory_anchors_t *blk)
{
    if (!blk) return false;
    if (blk->magic != FACTORY_MAGIC)     return false;
    if (blk->version != FACTORY_VERSION) return false;

    force_factory_anchors_t tmp;
    memcpy(&tmp, blk, sizeof(tmp));
    uint32_t saved = tmp.crc32;
    tmp.crc32 = 0;
    return (crc32_le(&tmp, sizeof(tmp)) == saved);
}

bool force_anchors_read(force_factory_anchors_t *out)
{
    if (!out) return false;
    const force_factory_anchors_t *flash = FA_ptr();
    if (!force_anchors_valid(flash)) return false;
    memcpy(out, flash, sizeof(*out));
    return true;
}

bool force_anchors_write(const force_factory_anchors_t *in)
{
    if (!in) return false;

    // refuse if already sealed
    const force_factory_anchors_t *flash = FA_ptr();
    if (force_anchors_valid(flash) && flash->sealed) return false;

    // normalize header + CRC to current constants
    force_factory_anchors_t w;
    memcpy(&w, in, sizeof(w));
    w.magic   = FACTORY_MAGIC;
    w.version = FACTORY_VERSION;
    w.sealed  = 0;       // writes always produce an unsealed block
    w.crc32   = 0;
    w.crc32   = crc32_le(&w, sizeof(w));

    if (!flash_write_factory(&w)) return false;
    return force_anchors_valid(FA_ptr());
}

bool force_anchors_lock(void)
{
    const force_factory_anchors_t *flash = FA_ptr();
    if (!force_anchors_valid(flash)) return false;
    if (flash->sealed) return true;   // already sealed

    force_factory_anchors_t w;
    memcpy(&w, flash, sizeof(w));
    w.sealed = 1;
    w.crc32  = 0;
    w.crc32  = crc32_le(&w, sizeof(w));

    if (!flash_write_factory(&w)) return false;
    return force_anchors_valid(FA_ptr()) && FA_ptr()->sealed == 1;
}


bool force_anchors_handle_op(uint8_t op,
                             const uint8_t *in_payload, uint8_t in_len,
                             uint8_t *out_buf, uint8_t *out_len)
{
    if (!out_buf || !out_len) return false;
    *out_len = 0;

    switch (op)
    {
    case OP_GET_FACTORY_ANCHORS: {
        force_factory_anchors_t blk;
        if (!force_anchors_read(&blk)) {
            // If not initialized/invalid, return a zeroed struct so GUI can warn.
            memset(&blk, 0, sizeof(blk));
        }
        memcpy(out_buf, &blk, sizeof(blk));
        *out_len = (uint8_t)sizeof(blk);
        return true;
    }

    case OP_SET_FACTORY_ANCHORS: {
        if (!in_payload || in_len != sizeof(force_factory_anchors_t)) {
            return false; // bad length ? let caller decide how to NAK
        }
        const force_factory_anchors_t *in = (const force_factory_anchors_t*)in_payload;
        bool ok = force_anchors_write(in);
        out_buf[0] = ok ? 1 : 0;   // 1=OK, 0=FAIL (e.g., sealed)
        *out_len = 1;
        return true;
    }

    case OP_LOCK_FACTORY_ANCHORS: {
        bool ok = force_anchors_lock();
        out_buf[0] = ok ? 1 : 0;
        *out_len = 1;
        return true;
    }

   
	}
}
