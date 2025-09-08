/**
  ******************************************************************************
  * @file           : device_info.h
  * @brief          : Header for device identification storage module providing
  *                   API for reading/writing device serial number, model, and
  *                   manufacturing date to persistent flash memory.
  * @project        : Invictus HOTAS Firmware
  * @author         : Invictus Cockpit Systems
  * @version        : 1.0.0
  * @date           : 2025-01-02
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
	
#ifndef DEVICE_INFO_H
#define DEVICE_INFO_H

#include <stdint.h>
#include <stdbool.h>  // Add this for bool type
#include "common_types.h"

void device_info_init(void);

bool device_info_handle_op(uint8_t op, const uint8_t *in_payload, uint8_t in_len,
                           uint8_t *out_buf, uint8_t *out_len);

extern device_info_t g_device_info;

#endif // DEVICE_INFO_H
