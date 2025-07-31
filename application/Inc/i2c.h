/**
  ******************************************************************************
  * @file           : i2c.h
  * @brief          : Header file for i2c.h                 
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
#ifndef __I2C_H__
#define __I2C_H__

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#define I2C_TIMEOUT		500

void I2C_Start(void);

int I2C_WriteBlocking(uint8_t dev_addr, uint8_t reg_addr, uint8_t * data, uint16_t length);
int I2C_ReadBlocking(	uint8_t dev_addr, uint8_t reg_addr, uint8_t * data, uint16_t length, uint8_t nack);

int I2C_WriteNonBlocking(uint8_t dev_addr, uint8_t * data, uint16_t length);
int I2C_ReadNonBlocking(uint8_t dev_addr, uint8_t reg_addr, uint8_t * data, uint16_t length, uint8_t nack);

#endif 	/* __I2C_H__ */

