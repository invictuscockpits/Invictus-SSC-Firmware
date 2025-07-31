/**
  ******************************************************************************
  * @file           : mcp320x.h
  * @brief          : Header for mcp320x.c file.
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
#ifndef __MCP320X_H__
#define __MCP320X_H__

#include "common_types.h"
#include "periphery.h"

#define MCP32xx_SPI_MODE						1

uint16_t MCP320x_GetData(sensor_t * sensor, uint8_t channel);
void MCP320x_StartDMA(sensor_t * sensor, uint8_t channel);
void MCP320x_StopDMA(sensor_t * sensor);

#endif 	/* __MCP320X_H__ */

