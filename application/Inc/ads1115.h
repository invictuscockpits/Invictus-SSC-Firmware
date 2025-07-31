/**
  ******************************************************************************
  * @file           : ads1115.h
  * @brief          : Header for ads1115.c file.
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
#ifndef __ADS1115_H__
#define __ADS1115_H__

#include "common_types.h"
#include "periphery.h"

#define ADS1115_I2C_ADDR_MIN				0x48
#define ADS1115_I2C_ADDR_MAX				0x4B

void ADS1115_Init(sensor_t * sensor);
int16_t ADS1115_GetData(sensor_t * sensor, uint8_t channel);

int ADS1115_ReadBlocking(sensor_t * sensor, uint8_t channel);
int ADS1115_SetMuxBlocking(sensor_t * sensor, uint8_t channel);

int ADS1115_StartDMA(sensor_t * sensor, uint8_t channel);
int ADS1115_SetMuxDMA(sensor_t * sensor, uint8_t channel);

#endif 	/* __ADS1115_H__ */

