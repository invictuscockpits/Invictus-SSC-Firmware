/**
  ******************************************************************************
  * @file           : force_anchors.h
  * @brief          : Header file for force_anchors.h                 
  * @project        Invictus HOTAS Firmware
  * @author         Invictus Cockpit Systems
  * @version        1.1.0
  * @date           2025-09-06
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
