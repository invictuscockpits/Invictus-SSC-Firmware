/**
  ******************************************************************************
  * @file           : tle5012.h
  * @brief          : Header for tle5012.c file.
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
#ifndef __TLE5012_H__
#define __TLE5012_H__

#include "common_types.h"
#include "periphery.h"

#ifndef M_PI
	#define M_PI							3.1415926535897932384626433832795
#endif

#define TLE5012_SPI_MODE			1

void TLE5012_StartDMA(sensor_t * sensor);
void TLE5012_StopDMA(sensor_t * sensor);
int TLE5012_GetAngle(sensor_t * sensor, float * angle);

#endif 	/* __TLE5012_H__ */

