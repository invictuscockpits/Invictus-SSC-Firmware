/**
  ******************************************************************************
  * @file           : shift_registers.h
  * @brief          : Header for shift_registers.c file.
  * @project        Invictus HOTAS Firmware
  * @author         Invictus Cockpit Systems
  * @version        1.0.0
  * @date           2025-07-30
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SHIFT_REGISTERS_H__
#define __SHIFT_REGISTERS_H__

#include "common_types.h"
#include "periphery.h"

#define SHIFTREG_TICK_DELAY			5 //changed from 1 to 5 to allow longer lead cables from sensor to board for those who need that.

void ShiftRegistersInit(dev_config_t * p_dev_config);
void ShiftRegistersGet (uint8_t * raw_button_data_buf, dev_config_t * p_dev_config, uint8_t * pos);

#endif 	/* __SHIFT_REGISTERS_H__ */

