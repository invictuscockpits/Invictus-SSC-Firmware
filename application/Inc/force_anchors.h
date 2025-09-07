/**
  ******************************************************************************
  * @file           : force_anchors.h
  * @brief          : Header file for force_anchors.h                 
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


uint32_t force_crc32(const void *data, uint32_t len);

bool force_anchors_valid(const force_factory_anchors_t *blk);
bool force_anchors_read(force_factory_anchors_t *out);
bool force_anchors_write(const force_factory_anchors_t *in);
bool force_anchors_lock(void);
bool force_anchors_unlock(void);  


bool force_anchors_handle_op(uint8_t op,
                             const uint8_t *in_payload, uint8_t in_len,
                             uint8_t *out_buf, uint8_t *out_len);

#ifdef __cplusplus
}
#endif
#endif /* __FORCE_ANCHORS_H__ */
