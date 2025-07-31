/**
  ******************************************************************************
  * @file           : mlx90363.h
  * @brief          : Header for mlx90363.c file.
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
#ifndef __MLX90363_H__
#define __MLX90363_H__

#include "common_types.h"
#include "periphery.h"

#define MLX90363_SPI_MODE			1

#define GET1_OPCODE 				0x13
#define NOP_OPCODE					0xD0

#define NULL_DATA					0x00
#define NOP_KEY						0xAA
#define GET_TIME_OUT 				0xFF

void MLX90363_Start(sensor_t * sensor);

int MLX90363_GetData(uint16_t * data, sensor_t * sensor, uint8_t channel);

void MLX90363_StartDMA(sensor_t * sensor);
void MLX90363_StopDMA(sensor_t * sensor);

#endif 	/* __MLX90363_H__ */

