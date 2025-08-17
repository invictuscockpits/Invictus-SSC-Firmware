/**
  ******************************************************************************
  * @file           : force_anchors.h
  * @brief          : Header file for force_anchors.h                 
  * @project        Invictus HOTAS Firmware
  * @author         Invictus Cockpit Systems
  * @version        1.0.0
  * @date           2025-08-16
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
  * This software has been carefully modified for a specific purpose.  It is not recommended for use outside of the Invictus HOTAS system.
  *
  ******************************************************************************
  */

#ifndef __FORCE_ANCHORS_H__
#define __FORCE_ANCHORS_H__

#include <stdint.h>
#include <stdbool.h>
#include "common_types.h"   // force_factory_anchors_t

#ifdef __cplusplus
extern "C" {
#endif

// CRC32 helper (LE, poly 0xEDB88320) – exposed for test tooling if needed
uint32_t force_crc32(const void *data, uint32_t len);

// Validate a RAM block against magic/version/CRC (CRC computed with crc32 field zeroed)
bool force_anchors_valid(const force_factory_anchors_t *blk);

// Read anchors from flash into *out. Returns true on success (valid block present).
bool force_anchors_read(force_factory_anchors_t *out);

// Write anchors to flash if not sealed. Rewrites header/CRC. Returns true on success.
bool force_anchors_write(const force_factory_anchors_t *in);

// Set sealed=1 and rewrite with updated CRC. Idempotent. Returns true on success.
bool force_anchors_lock(void);

// Handle DEV ops for force anchors (read,write,lock).
// Returns true if a reply payload was produced.
// - op: one of OP_GET_FACTORY_ANCHORS, OP_SET_FACTORY_ANCHORS, OP_LOCK_FACTORY_ANCHORS
// - in_payload/in_len: request payload (may be NULL/0 depending on op)
// - out_buf/out_len: reply payload (caller provides buffer; set *out_len on success)
bool force_anchors_handle_op(uint8_t op,
                             const uint8_t *in_payload, uint8_t in_len,
                             uint8_t *out_buf, uint8_t *out_len);

#ifdef __cplusplus
}
#endif
#endif /* __FORCE_ANCHORS_H__ */
