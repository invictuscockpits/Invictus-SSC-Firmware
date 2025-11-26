/**
  ******************************************************************************
  * @file           : mcp320x.h
  * @brief          : Header for mcp320x.c file.
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
#ifndef __MCP320X_H__
#define __MCP320X_H__

#include "common_types.h"
#include "periphery.h"

#define MCP32xx_SPI_MODE						1

uint16_t MCP320x_GetData(sensor_t * sensor, uint8_t channel);
void MCP320x_StartDMA(sensor_t * sensor, uint8_t channel);
void MCP320x_StopDMA(sensor_t * sensor);

#endif 	/* __MCP320X_H__ */

