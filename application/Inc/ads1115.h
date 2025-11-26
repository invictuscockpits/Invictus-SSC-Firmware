/**
  ******************************************************************************
  * @file           : ads1115.h
  * @brief          : Header for ads1115.c file.
  * @project        Invictus HOTAS Firmware
  * @author         Invictus Cockpit Systems
  * @version        1.0.1
  * @date           2025-11-24
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
#ifndef __ADS1115_H__
#define __ADS1115_H__

#include "common_types.h"
#include "periphery.h"

#define ADS1115_I2C_ADDR_MIN				0x48
#define ADS1115_I2C_ADDR_MAX				0x4B

void ADS1115_Init(sensor_t * sensor, dev_config_t * p_dev_config);
int16_t ADS1115_GetData(sensor_t * sensor, uint8_t channel);

int ADS1115_ReadBlocking(sensor_t * sensor, uint8_t channel);
int ADS1115_SetMuxBlocking(sensor_t * sensor, uint8_t channel, dev_config_t * p_dev_config);

int ADS1115_StartDMA(sensor_t * sensor, uint8_t channel);
int ADS1115_SetMuxDMA(sensor_t * sensor, uint8_t channel, dev_config_t * p_dev_config);

#endif 	/* __ADS1115_H__ */

