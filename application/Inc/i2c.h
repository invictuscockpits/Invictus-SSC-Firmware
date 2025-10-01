/**
  ******************************************************************************
  * @file           : i2c.h
  * @brief          : Header file for i2c.h                 
  * @project        Invictus HOTAS Firmware
  * @author         Invictus Cockpit Systems
  * @version        1.2.1
  * @date           2025-09-09
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
  * This software has been carefully modified for a specific purpose.  
	* It is not recommended for use outside of the Invictus HOTAS system.
  *
  ******************************************************************************
  */

#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"
#include "common_types.h"     // for pin_t, I2C_SCL / I2C_SDA enums
#include "common_defines.h"   // for USED_PINS_NUM 

// ===== Optional: timeout guard (kept same default as your code) =====
#ifndef I2C_TIMEOUT
#define I2C_TIMEOUT 100000UL
#endif



/**
 * @brief Auto-select I2C bus by inspecting configured pins array.
 *        If PB6==I2C_SCL and PB7==I2C_SDA -> selects I2C1, else I2C2.
 * @param pins  pointer to array of pin function enums (config.pins[])
 * @param count length of the pins array
 */
void I2C_SelectBusFromPins(const pin_t *pins, size_t count);

/**
 * @brief Initialize the currently selected I2C peripheral (default I2C1).
 */
void I2C_Start(void);

/** Blocking write: dev_addr/reg_addr + payload. 0 on success, <0 on timeout/error. */
int I2C_WriteBlocking(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t length);

/** Blocking read: write reg, then read payload. 0 on success, <0 on timeout/error. */
int I2C_ReadBlocking(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t length, uint8_t nack);

/** Non-blocking write via DMA. 0 on success, <0 on timeout/error. */
int I2C_WriteNonBlocking(uint8_t dev_addr, uint8_t *data, uint16_t length);

/** Non-blocking read via DMA. 0 on success, <0 on timeout/error. */
int I2C_ReadNonBlocking(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t length, uint8_t nack);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */
