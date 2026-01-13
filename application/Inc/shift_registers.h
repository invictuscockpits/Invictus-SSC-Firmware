/**
  ******************************************************************************
  * @file           : shift_registers.h
  * @brief          : Header for shift_registers.c file.
  * @project        Invictus HOTAS Firmware
  * @author         Invictus Cockpit Systems
  * @version        1.0.0
  * @date           2025-07-30
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SHIFT_REGISTERS_H__
#define __SHIFT_REGISTERS_H__

#include "common_types.h"
#include "periphery.h"

#define SHIFTREG_TICK_DELAY			20 // Increased for reliable operation with long cables and CD4021 propagation delay

void ShiftRegistersInit(dev_config_t * p_dev_config);
void ShiftRegistersGet (uint8_t * raw_button_data_buf, dev_config_t * p_dev_config, uint8_t * pos);

#endif 	/* __SHIFT_REGISTERS_H__ */

