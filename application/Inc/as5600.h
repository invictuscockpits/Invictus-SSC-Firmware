/**
  ******************************************************************************
  * @file           : as5600.h
  * @brief          : Header for as5600.c file.
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
#ifndef __AS5600_H__
#define __AS5600_H__

#include "common_types.h"
#include "periphery.h"

#define AS5600_I2C_ADDR				0x36

void AS5600_Init(sensor_t * sensor, uint16_t min, uint16_t max);
int16_t AS5600_GetRawData(sensor_t * sensor);
int16_t AS5600_GetScaledData(sensor_t * sensor);
int AS5600_ReadBlocking(sensor_t * sensor);
int AS5600_StartDMA(sensor_t * sensor);

#endif 	/* __AS5600_H__ */

